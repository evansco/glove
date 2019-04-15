make monitor | tee out.dat
cat out.dat | grep "GLOVE: gyro:" | sed 's/.*GLOVE: gyro: (\([-[:digit:]]*, [-[:digit:]]*, [-[:digit:]]*\)).*/\1/g' > gyro.dat
cat out.dat | grep "GLOVE: accel:" | sed 's/.*GLOVE: accel: (\([-[:digit:]]*, [-[:digit:]]*, [-[:digit:]]*\)).*/\1/g' > accel.dat
cat out.dat | grep "GLOVE: X:" | sed 's/.*GLOVE: X: \([0-9]*\)\tY: \([0-9]*\).*/\1, \2/g' > pos.dat
python plot.py
