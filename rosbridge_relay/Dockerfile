FROM python:3.11-slim

WORKDIR /app

# 依存パッケージをインストール
RUN pip install --no-cache-dir websockets

# スクリプトをコピー
COPY relay.py /app/

# 実行権限を付与
RUN chmod +x /app/relay.py

# 出力をバッファリングしない
ENV PYTHONUNBUFFERED=1

# デフォルトのエントリポイント
ENTRYPOINT ["python3", "-u", "/app/relay.py"]

# デフォルト引数（上書き可能）
CMD ["--rosbridge", "ws://rosbridge:9090", "--udp-host", "host.docker.internal", "--udp-port", "9090"]
