## install

**absl**

```shell
git clone https://github.com/abseil/abseil-cpp.git
cd abseil-cpp
mkdir build && cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j8
sudo make install
```



**qp-oases**

```shell
git clone https://github.com/startcode/qp-oases.git
cd qp-oases
mkdir build && cd build
cmake ..
make -j8 CPPFLAGS="-Wall -pedantic -Wshadow -Wfloat-equal -O3 -Wconversion -Wsign-conversion -fPIC -DLINUX -DSOLVER_NONE -D__NO_COPYRIGHT__"
sudo make install
```



**osqp**

```shell
wget https://github.com/oxfordcontrol/osqp/archive/v0.5.0.tar.gz
tar xzf v0.5.0.tar.gz
cd osqp-0.5.0
wget https://github.com/oxfordcontrol/qdldl/archive/v0.1.4.tar.gz
tar xzf v0.1.4.tar.gz --strip-components=1 -C ./lin_sys/direct/qdldl/qdldl_sources
rm -rf v0.1.4.tar.gz
mkdir build && cd build
cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX="${SYSROOT_DIR}" -DCMAKE_BUILD_TYPE=Release
make -j8 && make install 
cd ../.. && rm -rf v0.5.0.tar.gz
ldconfig
```

