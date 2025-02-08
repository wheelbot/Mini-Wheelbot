FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

# toolchain
RUN apt-get update \
   && apt-get install -y \
      build-essential \
      libboost-system-dev \
      cmake \
      curl \
      make \
      python3 \
      python3-distutils \
      apt-utils \
      software-properties-common \
      dpkg \
      wget \
      git \
      iputils-ping \
   && rm -rf /var/lib/apt/lists/*

# realtime utils
RUN apt-get update \
   && apt-get install -y \
      rt-tests \
   && rm -rf /var/lib/apt/lists/*

# modern gcc
RUN add-apt-repository ppa:ubuntu-toolchain-r/test \
   && apt-get update \
   && apt-get install -y \
      gcc-13 \
      g++-13 \
   && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
   && apt-get install -y \
      libeigen3-dev \
   && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100 \
   && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100 \
   && update-alternatives --install /usr/bin/cpp cpp-bin /usr/bin/cpp-13 100 \
   && update-alternatives --set g++ /usr/bin/g++-13 \
   && update-alternatives --set gcc /usr/bin/gcc-13 \
   && update-alternatives --set cpp-bin /usr/bin/cpp-13

RUN apt-get update \
   && apt-get install -y \
      openssh-server \
   && rm -rf /var/lib/apt/lists/*

WORKDIR /
RUN apt-get update \
   && apt-get install -y \
      libaio1 \
      libavahi-client3 \
      libavahi-common3 \
      libusb-1.0-0 \
      libserialport0 \
   && rm -rf /var/lib/apt/lists/*
RUN wget https://github.com/analogdevicesinc/libiio/releases/download/v0.25/libiio-0.25.gb6028fd-Ubuntu-arm64v8.deb
RUN dpkg -i libiio-0.25.gb6028fd-Ubuntu-arm64v8.deb

WORKDIR /
RUN git clone --branch v3.3.1 http://github.com/odygrd/quill.git \
   && cd quill \
   && mkdir build \
   && cd build \
   && cmake .. \
   && make install -j4


RUN git clone --depth 1 --branch v1.0.6 https://github.com/cameron314/readerwriterqueue \
   && cd readerwriterqueue \
   && mkdir build \
   && cd build \
   && cmake .. \
   && make install -j4


RUN apt-get update \
   && apt-get install -y \
      build-essential \
      cmake \
      protobuf-compiler \
      libprotobuf-dev \
      libgtest-dev \
      libbenchmark-dev \
   && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
   && apt-get install -y \
      libgpiod-dev \
   && rm -rf /var/lib/apt/lists/*


WORKDIR /wheelbot-lib
#COPY src src
#COPY include include
#COPY CMakeLists.txt .
#RUN ls

#RUN mkdir build && cd build && cmake .. && make -j4

########################################################
# Remote debugging and login in
########################################################

RUN mkdir /var/run/sshd
RUN echo 'root:root' | chpasswd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# SSH login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile

RUN sed -i 's/\(^Port\)/#\1/' /etc/ssh/sshd_config && echo Port 2233 >> /etc/ssh/sshd_config

# expose ssh port
EXPOSE 2233

# expose input server port
EXPOSE 8888

RUN useradd -ms /bin/bash debugger
RUN echo 'debugger:pwd' | chpasswd

CMD ["/usr/sbin/sshd", "-D"]
