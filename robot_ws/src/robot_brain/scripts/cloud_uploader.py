#!/usr/bin/env python3
"""
云端数据上传客户端 — 沿用 doubao_client.py 的 urllib.request 模式。

支持:
  - JSON 统计报告上传
  - 缩略图 base64 上传
  - 离线缓存 (上传失败时存本地 JSON)
  - 批量上报

独立测试:
  python3 cloud_uploader.py --test
  构造假数据 POST 到火山引擎, 验证连通性
"""
import json
import base64
import urllib.request
import urllib.error
import time
import os
import sys
import io
from dataclasses import dataclass, field, asdict


@dataclass
class OrchardReport:
    """单次果园巡检统计报告。"""
    robot_id: str = 'RDK-X5-001'
    timestamp: str = ''
    orchard_id: str = 'block-1'
    row: int = 0
    total_apples: int = 0
    good: int = 0
    bad: int = 0
    unripe: int = 0
    unknown: int = 0
    good_ratio: float = 0.0
    location: dict = field(default_factory=lambda: {'x': 0.0, 'y': 0.0})
    thumbnail_base64: str = ''

    @classmethod
    def from_stats(cls, robot_id, orchard_id, row, location, summary, thumbnail=None):
        """从统计字典构建报告。"""
        total = summary.get('total', 0)
        good = summary.get('good', 0)
        bad = summary.get('bad', 0)
        unripe = summary.get('unripe', 0)
        unknown = summary.get('unknown', 0)
        ratio = good / max(total, 1)

        thumb_b64 = ''
        if thumbnail is not None:
            thumb_b64 = CloudUploader.encode_image(thumbnail, quality=60)

        return cls(
            robot_id=robot_id,
            timestamp=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            orchard_id=orchard_id,
            row=row,
            total_apples=total,
            good=good,
            bad=bad,
            unripe=unripe,
            unknown=unknown,
            good_ratio=round(ratio, 4),
            location=location,
            thumbnail_base64=thumb_b64,
        )


class CloudUploader:
    """火山引擎 DataArk 数据上传客户端。

    复用与 DoubaoClient 相同的 urllib.request 模式:
      urlopen(Request(url, data=json.dumps(body).encode(),
             headers={'Content-Type':'application/json', 'Authorization':'Bearer ...'}))
    """

    def __init__(self, api_key='', endpoint='', dataset_id='orchard-apple-quality',
                 offline_cache_dir='/tmp/orchard_cache'):
        """
        Args:
            api_key:     火山引擎 API Key (与豆包共用)
            endpoint:    数据上传端点 URL
            dataset_id:  数据集 ID
            offline_cache_dir: 离线缓存目录 (上传失败时暂存)
        """
        self.api_key = api_key
        self.endpoint = endpoint.rstrip('/') if endpoint else ''
        self.dataset_id = dataset_id
        self.cache_dir = offline_cache_dir
        self.retry_count = 3
        self.retry_delay = 2.0  # 秒

        if not self.endpoint:
            print('[CloudUploader] No endpoint configured — uploads disabled')
        else:
            print(f'[CloudUploader] {self.endpoint}')

    @staticmethod
    def encode_image(image, quality=60):
        """OpenCV BGR image → base64 JPEG 字符串。"""
        import cv2
        import numpy as np
        if image is None:
            return ''
        # 缩放到 320 宽
        h, w = image.shape[:2]
        if w > 320:
            scale = 320.0 / w
            image = cv2.resize(image, (320, int(h * scale)))

        _, buf = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return base64.b64encode(buf).decode('utf-8')

    def upload_report(self, report: OrchardReport) -> bool:
        """上传统计报告 + 缩略图。

        Returns:
            True:  上传成功
            False: 失败 (已尝试重试 + 本地缓存)
        """
        if not self.endpoint:
            print('[CloudUploader] Upload skipped (no endpoint)')
            return False

        body = asdict(report)

        for attempt in range(self.retry_count):
            try:
                data = json.dumps(body).encode('utf-8')
                url = f'{self.endpoint}/datasets/{self.dataset_id}/records'
                headers = {
                    'Content-Type': 'application/json',
                    'Authorization': f'Bearer {self.api_key}',
                }
                req = urllib.request.Request(url, data=data, headers=headers,
                                             method='POST')
                with urllib.request.urlopen(req, timeout=30) as resp:
                    result = json.loads(resp.read().decode())
                    print(f'[CloudUploader] OK — row={report.row} '
                          f'good={report.good}/{report.total_apples}')
                    return True

            except urllib.error.HTTPError as e:
                err_body = e.read().decode()[:200] if e.fp else ''
                print(f'[CloudUploader] HTTP {e.code} (attempt {attempt+1}): '
                      f'{err_body}')
            except Exception as e:
                print(f'[CloudUploader] Error (attempt {attempt+1}): {e}')

            if attempt < self.retry_count - 1:
                time.sleep(self.retry_delay * (attempt + 1))

        # 所有重试失败 → 本地缓存
        self._cache_locally(body)
        return False

    def upload_batch(self, reports: list[OrchardReport]) -> int:
        """批量上传, 返回成功数。"""
        ok_count = 0
        for report in reports:
            if self.upload_report(report):
                ok_count += 1
        print(f'[CloudUploader] Batch: {ok_count}/{len(reports)} uploaded')
        return ok_count

    def _cache_locally(self, body: dict):
        """上传失败时缓存到本地 JSON 文件。"""
        os.makedirs(self.cache_dir, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.cache_dir, f'report_{ts}.json')
        try:
            with open(path, 'w') as f:
                json.dump(body, f, ensure_ascii=False, indent=2)
            print(f'[CloudUploader] Cached to {path}')
        except Exception as e:
            print(f'[CloudUploader] Cache write error: {e}')

    def flush_cache(self) -> int:
        """重传本地缓存的报告。返回成功重传数量。"""
        if not os.path.isdir(self.cache_dir):
            return 0

        cached = [f for f in os.listdir(self.cache_dir) if f.endswith('.json')]
        ok = 0
        for fname in sorted(cached):
            path = os.path.join(self.cache_dir, fname)
            try:
                with open(path) as f:
                    body = json.load(f)
                data = json.dumps(body).encode('utf-8')
                url = f'{self.endpoint}/datasets/{self.dataset_id}/records'
                headers = {
                    'Content-Type': 'application/json',
                    'Authorization': f'Bearer {self.api_key}',
                }
                req = urllib.request.Request(url, data=data, headers=headers,
                                             method='POST')
                with urllib.request.urlopen(req, timeout=30):
                    os.remove(path)
                    ok += 1
                    print(f'[CloudUploader] Flushed: {fname}')
            except Exception as e:
                print(f'[CloudUploader] Flush failed {fname}: {e}')
        print(f'[CloudUploader] Flush done: {ok}/{len(cached)}')
        return ok


# ─── 独立测试 ─────────────────────────────────────────────
def main():
    import argparse
    parser = argparse.ArgumentParser(description='CloudUploader 独立测试')
    parser.add_argument('--test', action='store_true',
                        help='构造假数据测试上传')
    parser.add_argument('--api-key', default='',
                        help='火山引擎 API Key')
    parser.add_argument('--endpoint', default='',
                        help='数据上传端点 URL')
    parser.add_argument('--flush', action='store_true',
                        help='重传本地缓存的报告')
    args = parser.parse_args()

    uploader = CloudUploader(
        api_key=args.api_key,
        endpoint=args.endpoint,
    )

    if args.flush:
        print('Flushing cached reports...')
        uploader.flush_cache()
        return

    if args.test:
        print('=== CloudUploader Test ===')

        # 构造假统计
        summary = {'total': 45, 'good': 38, 'bad': 5, 'unripe': 2, 'unknown': 0}

        # 生成假缩略图 (纯色)
        import numpy as np
        thumb = np.zeros((240, 320, 3), dtype=np.uint8)
        thumb[:, :] = (0, 200, 0)  # 绿色底

        report = OrchardReport.from_stats(
            robot_id='RDK-X5-test',
            orchard_id='test-block',
            row=1,
            location={'x': 1.5, 'y': 3.2},
            summary=summary,
            thumbnail=thumb,
        )

        print(f'Report: row={report.row} total={report.total_apples} '
              f'good={report.good}/{report.total_apples} '
              f'({report.good_ratio:.1%})')
        print(f'Thumbnail: {len(report.thumbnail_base64)} chars base64')

        if args.api_key and args.endpoint:
            ok = uploader.upload_report(report)
            print(f'Upload: {"OK" if ok else "FAILED"}')
        else:
            print('No API key/endpoint — dry-run only')
            print('Usage: python3 cloud_uploader.py --test '
                  '--api-key YOUR_KEY --endpoint https://...')
    else:
        print('Usage: python3 cloud_uploader.py --test '
              '--api-key YOUR_KEY --endpoint https://...')
        print('   or: python3 cloud_uploader.py --flush')


if __name__ == '__main__':
    main()
