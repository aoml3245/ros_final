# 얼굴 인식을 통한 사용자 트래킹 핸드폰 거치대 -Phone Mount with Face Recognition for User Tracking

> 주의!
현재 rosflask Dockerfile이 jetson nano를 위한 이미지를 사용합니다.
이 외의 디바이스에서 구동 시 이미지 버전을 바꿔주세요.
> 

- 구동 방법(Jetson Nano)

```bash
sudo apt update
sudo apt upgrade
sudo apt install dnsmasq hostapd ufw

# port open
sudo ufw enable
sudo ufw allow 22
sudo ufw allow 443
sudo ufw allow 80

# AP 모드 on
git clone https://github.com/oblique/create_ap
cd create_ap
sudo make install
ip link show
sudo reboot
sudo create_ap wlan0 eth0 Nano
echo -e '#!/bin/bash\nsudo rfkill unblock all\nsudo create_ap --no-virt -w 2 wlan0 eth0 Nano 12341234' | sudo tee /etc/init.d/hotspot.sh > /dev/null
echo -e '[Unit]\nDescription=Create WiFi hotspot using create_ap\nAfter=network.target\n\n[Service]\nExecStart=/usr/bin/sudo /etc/init.d/hotspot.sh\nRestart=on-failure\nRestartSec=30\nExecStartPre=/bin/sleep 30\n\n[Install]\nWantedBy=multi-user.target' | sudo tee /etc/systemd/system/hotspot.service > /dev/null
sudo chmod +x /etc/init.d/hotspot.sh
sudo systemctl daemon-reload
sudo systemctl enable hotspot.service
sudo systemctl start hotspot.service
```

```bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
# Add the repository to Apt sources:
echo   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
$(. /etc/os-release && echo "$VERSION_CODENAME") stable" |   sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove -y $pkg; done
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo apt autoremove -y
sudo docker run hello-world
sudo usermod -aG docker $USER
```

```bash
#핫스팟 내 노트북, 젯슨의 ip확인

#git clone 해당 레파지토리

cd ros_final
docker compose up --build
```

- 기타 참고 자료
    - docker nginx self signing 진행(ssl)
        
        https://gist.github.com/ykarikos/06879cbb0d80828fe96445b504fa5b60
        

- 수업에서 나눠준 보드에 marin firmware 설치
    - vscose 설치
    - platformio 익스텐션 설치
    - 디버깅 모드로 보드와 컴퓨터 연결
    - 업로드(보드 업로드 모드 켜주기)
- 보드를 nano 핫스팟 연결해준다.
- 스마트폰으로 nano 핫스팟에 연결해준다.
- nano ip로 접속
