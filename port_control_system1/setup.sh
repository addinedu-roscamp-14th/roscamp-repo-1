#!/bin/bash

echo "========================================"
echo "Port Control System 환경 자동 설정 스크립트"
echo "========================================"
echo ""

# 1. 시스템 패키지 설치
echo "[1/3] 시스템 필수 프로그램 설치 중 (tmux, gnome-terminal, openssh-client)..."
echo "관리자(sudo) 권한이 필요할 수 있습니다."
sudo apt update
sudo apt install -y tmux gnome-terminal openssh-client

# 2. 파이썬 패키지 설치
echo ""
echo "[2/3] 파이썬 필수 라이브러리 설치 중..."
pip install -r requirements.txt

# 3. Ollama 설치 및 기본 모델 다운로드
echo ""
echo "[3/3] Ollama AI 엔진 설치 및 모델 세팅..."
if ! command -v ollama &> /dev/null
then
    echo "Ollama가 발견되지 않았습니다. 설치 스크립트를 실행합니다..."
    curl -fsSL https://ollama.com/install.sh | sh
else
    echo "Ollama가 이미 설치되어 있습니다."
fi

# 코드에 지정된 기본 모델 다운로드
DEFAULT_MODEL="gemma4:31b"
echo "기본 모델(${DEFAULT_MODEL}) 다운로드/업데이트를 진행합니다..."
ollama pull ${DEFAULT_MODEL}

echo ""
echo "========================================"
echo "모든 환경 설정이 성공적으로 완료되었습니다!"
echo "Ollama 서버가 켜진 상태에서 파이썬 파일을 실행하시면 됩니다."
echo "========================================"
