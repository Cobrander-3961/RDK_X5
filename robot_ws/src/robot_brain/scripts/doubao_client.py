#!/usr/bin/env python3
"""
豆包 Vision Pro API 客户端。

支持:
  - 图片 + 文字 → 决策 (chat_with_image)
  - 纯文字 → 对话 (chat)
"""
import json, base64, urllib.request, urllib.error
from pathlib import Path


class DoubaoClient:
    """火山引擎豆包大模型 API 封装。"""

    def __init__(self, api_key: str, model: str = "ep-20260609152524-hhxtc",
                 endpoint: str = "https://ark.cn-beijing.volces.com/api/v3",
                 max_tokens: int = 1024, temperature: float = 0.7):
        self.api_key = api_key
        self.model = model
        self.endpoint = endpoint.rstrip("/")
        self.max_tokens = max_tokens
        self.temperature = temperature

    def _request(self, messages: list) -> dict:
        url = f"{self.endpoint}/chat/completions"
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }
        body = {
            "model": self.model,
            "messages": messages,
            "max_tokens": self.max_tokens,
            "temperature": self.temperature
        }
        data = json.dumps(body).encode("utf-8")
        req = urllib.request.Request(url, data=data, headers=headers, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=30) as resp:
                return json.loads(resp.read().decode())
        except urllib.error.HTTPError as e:
            return {"error": str(e), "body": e.read().decode()}

    @staticmethod
    def _image_to_base64(image_path: str) -> str:
        with open(image_path, "rb") as f:
            return base64.b64encode(f.read()).decode()

    def chat(self, user_text: str, system_prompt: str = "") -> str:
        """纯文本对话。"""
        messages = []
        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})
        messages.append({"role": "user", "content": user_text})
        resp = self._request(messages)
        if "error" in resp:
            return f"[ERROR] {resp['error']}"
        return resp["choices"][0]["message"]["content"]

    def chat_with_image(self, user_text: str, image_path: str,
                        system_prompt: str = "") -> str:
        """
        图片 + 文字多模态对话 (Vision Pro)。

        Args:
            user_text: 自然语言指令
            image_path: 图片文件路径
            system_prompt: 系统提示词

        Returns:
            模型回复文本
        """
        image_b64 = self._image_to_base64(image_path)
        image_url = f"data:image/jpeg;base64,{image_b64}"

        messages = []
        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})

        messages.append({
            "role": "user",
            "content": [
                {"type": "image_url", "image_url": {"url": image_url}},
                {"type": "text", "text": user_text}
            ]
        })

        resp = self._request(messages)
        if "error" in resp:
            return f"[ERROR] {resp['error']}"
        return resp["choices"][0]["message"]["content"]


def main():
    """测试: 纯文本 + 图片对话。"""
    import sys
    if len(sys.argv) < 2:
        print("Usage: doubao_client.py <api_key> [image_path]")
        sys.exit(1)

    api_key = sys.argv[1]
    client = DoubaoClient(api_key=api_key)

    if len(sys.argv) > 2:
        # 图片模式
        result = client.chat_with_image(
            "描述这张图片里有什么，机械臂应该怎么抓取？",
            sys.argv[2],
            system_prompt="你是机器人控制专家，回答要简洁。目标物体用[x,y,z]格式给出相对位置。"
        )
    else:
        result = client.chat("你好，介绍一下你自己")

    print(f"Reply: {result}")


if __name__ == "__main__":
    main()
