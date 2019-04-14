make monitor | tee out.dat
cat out.dat | grep "GLOVE: gyro:" | sed 's/.*GLOVE: gyro: (\([-[:digit:]]*, [-[:digit:]]*, [-[:digit:]]*\)).*/\1/g' > gyro.dat
cat out.dat | grep "GLOVE: accel:" | sed 's/.*GLOVE: accel: (\([-[:digit:]]*, [-[:digit:]]*, [-[:digit:]]*\)).*/\1/g' > accel.dat
python plot.py
