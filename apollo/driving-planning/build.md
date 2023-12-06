**eigen**

```shell
sudo apt install libeigen3-dev
```



**osqp**

*注意版本*

```shell
git clone --recursive https://github.com/oxfordcontrol/osqp
cd osqp
git checkout v0.6.3

# 增加qdldl库
cd lin_sys/direct/qdldl/
git clone git@github.com:osqp/qdldl.git
mv qdldl/* .
cd ../../..

mkdir build && cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j8
sudo make install
sudo ldconfig
```



**osqp-eigen**

```shell
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
make -j8
sudo make install
sudo ldconfig
```

