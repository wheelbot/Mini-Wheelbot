# Build and export dockerfile w/ cross compile
```bash
docker buildx create --use
docker buildx build --platform=linux/arm64 -t wheelbot-lib:latest --load .
docker save -o ../wheelbot-lib.container.tar wheelbot-lib:latest
```

Now, build the Buildroot image which copies over the container.
Container should be imported and started automatically upon first boot.

# Manual setup of container on the Mini Wheelbot
Contents of this repo should be at located at `/root/wheelbot-lib` on the Mini Wheelbot Pi CM4.
Run
```bash
docker compose up --build -d
```
to start the development docker container.
This will mount `./` (this repo) into the container at `/wheelbot-lib`.
The container has an ssh server at port `2233` and mounts all devices.

The docker compose file specifies restart to `always`, the container should therefore automatically start upon reboot, so you only need to do this once.

# Mounting remote `wheelbot-lib` folder on your laptop
From your laptop, you can mount the the `/root/wheelbot-lib` directory using:
```bash
sshfs -o port=22 root@192.168.0.213:/root/wheelbot-lib ~/mnt/wheelbot-lib
```

Now, you can use the IDE of your choice on your machine to develop on the Mini Wheelbot.

# Access the container for building
To compile and run the application inside the docker container
```bash
ssh -p 2233 root@192.168.0.213
```

# Building and running the Mini Wheelbot application
In a shell on the docker on the Mini Wheelbot (aka `port 2233`), go to build directory with
```
cd /wheelbot-lib/build
```
and build the software with
```
make -j8
```

In case you changed the `CMakeLists.txt`, you want to run
```
cmake ..
make -j8
```

To run the application
```
./Main
```

After you ran the application, you can find log files inside the folder
```
wheelbot-lib/log
```

# Running the Mini Wheelbot software: keyboard yaw control
In a shell on the docker on the Mini Wheelbot (aka `port 2233`), go to the `/wheelbot-lib/build` folder.
Make sure, that you are on the `jsonsetpoint` branch (should be the case for `wheelbot-beta-3`).
Now you can start the Mini Wheelbot application by:
```
./Main
```
Balance the robot on it's wheel until it stands.

On your laptop, in this repo, go to the `scripts/keyboardcontrol` folder.
You can run the keyboard input program by:
```
python main.py
```
Now you can use the `A`, `W`, `S`, and `D` keys to drive the robot around.

When closing the application, first shut down the Python window on your laptop, then close the main application on the Mini Wheelbot.
