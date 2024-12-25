At boot time, `buttonWatchdog.py` is executed by the root user.
Its purpose is to monitor the Button pin and shutdown the board in case of a long press.

In case something is not working, please go through the following file:

- etc/systemd/system/shutdown_with_button.service

By default it contains the following configuration:

```bash
[Unit]
Description=Run Python script to monitor button and shutdown on long press
After=network.target

[Service]
ExecStart=/usr/bin/python /home/jetbot/watchdog/livenessWatchdog/buttonWatchdog.py
Restart=on-failure
RestartSec=5
User=root
Group=root
WorkingDirectory=/home/jetbot/watchdog/livenessWatchdog
StandardOutput=journal
StandardError=journal
Environment="PYTHONPATH=/home/jetbot/.local/lib/python2.7/site-packages:/usr/lib/python2.7/site-packages"
[Install]
WantedBy=multi-user.target
```

If any changes are made, go through the following commands to restart and check the service:

```bash
sudo systemctl daemon-restart
sudo systemctl restart shutdown_with_button.service
sudo systemctl enable shutdown_with_button.service
sudo systemctl status shutdown_with_button.service
```
