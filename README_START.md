시작 스크립트 및 사용법

목적
- tmux 세션으로 제공된 ROS/노드/스크립트를 한 번에 띄우기 위해 `scripts/start_all_tmux.sh`를 제공합니다.

사용법

1. 실행 권한 부여:

```bash
chmod +x scripts/start_all_tmux.sh
```

2. 스크립트 실행:

```bash
./scripts/start_all_tmux.sh
```

3. tmux에 접속:

```bash
tmux attach -t poter_ws
```

창 리스트(스크립트에서 생성됨)
- `env`: 공통 환경 설정 로드
- `arm_camera`: 로봇팔 카메라 노드
- `arm_rectify`: 로봇팔 보정 (image_proc)
- `arm_server`: 로봇팔 서버

참고: 탑뷰(`topview_*`)와 `auto_pick`은 별도 창에서 수동으로 실행하세요.

카메라 인덱스 자동 선택 방법:
- 기본적으로 스크립트는 `CAMERA_IDX=0` (USB2_0Camera)를 사용합니다.
- HD 웹캠을 사용하려면 다음처럼 환경변수를 설정하고 실행하세요:

```bash
CAMERA_IDX=1 ./scripts/start_all_tmux.sh
```

또는 수동으로 카메라 노드를 실행할 때 `-p camera:=0` 또는 `-p camera:=1`을 사용하고, 해당 카메라에 맞는 캘리브 파일이 자동으로 선택됩니다.

문제 해결 팁
- `/dev/ttyUSB0`가 없다는 오류가 나오면 로봇암의 USB 연결을 확인하세요. `dmesg | tail`로 최근 USB 연결 로그 확인 가능.
- tmux가 없으면 설치:

```bash
sudo apt update
sudo apt install tmux
```

- 개별 프로세스를 수동으로 실행하려면 각 창에서 환경을 로드한 뒤 명령을 직접 실행하세요.

도움이 더 필요하면 알려주세요.