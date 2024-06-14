# ros_final

~~jetsonboard에 20.04.버전 다운(기존에 사용하던 버전임)~~
용량 문제로 공식 지원 18.04 버전으로 교체

```bash
sudo apt install python3-dev
git clone https://github.com/jetsonworld/jetson-fan-ctl.git
cd jetson-fan-ctl
sudo sh install.sh
sudo nano /etc/automagic-fan/config.json
sudo reboot
```

```bash
sudo apt update
sudo apt upgrade
sudo apt install dnsmasq hostapd ufw
sudo ufw enable
sudo ufw allow 22
sudo ufw allow 443
sudo ufw allow 80
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

docker nginx self signing 진행(ssl)

https://gist.github.com/ykarikos/06879cbb0d80828fe96445b504fa5b60

핫스팟 내 노트북, 젯슨의 ip확인

front: react 홈페이지
rosflask: ros+ flask 서버
docker compose 테스트

https://github.com/aoml3245/ros_final.git

git clone 해서 해당 폴더 내에서 docker compose up —build 실행

젯슨 보드에서 docker  실행하고 테스트

수업에서 나눠준 보드에 marin firmware 설치

viscose 설치

platform io

디버깅 모드로 보드와 컴퓨터 연결

업로드(보드 업로드 모드 켜주기)

보드 틀고 nano에 와이파이 연결해준다.

테스트
`G1 X1000 Y1000 F3000`
