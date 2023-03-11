#setup python 3.10
wget https://www.python.org/ftp/python/3.10.7/Python-3.10.7.tgz
apt install build-essential zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev
apt-get install libffi-dev libreadline-gplv2-dev libncursesw5-dev libssl-dev libsqlite3-dev tk-dev libgdbm-dev libc6-dev libbz2-dev
tar -xzvf Python-3.10.7.tgz 
cd Python-3.10.7/
./configure --enable-optimizations
make install
ln -sf $(which python3) /usr/bin/python

#setup project
# sudo -u arsis curl -sSL https://install.python-poetry.org | python3 -
# export PATH="/home/arsis/.local/bin:$PATH"
# poetry install  --no-interaction --no-ansi --no-root
python -m pip install pyserial fastapi uvicorn
echo @reboot sh /home/arsis/ARSIS-6/ARSIS-Devices/VisionKit/control_endpoint/run.sh > /etc/cron.d/serverstart