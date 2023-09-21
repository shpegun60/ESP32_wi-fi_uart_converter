INCLUDEPATH += $$PWD
DEPENDPATH += $$PWD	


include($$PWD/kalman_filter/kalman.pri)
include($$PWD/ahrs/ahrs.pri)
include($$PWD/matrix/matrix.pri)
include($$PWD/quaternion/quaternion.pri)
include($$PWD/smart_assert/smart_assert.pri)
include($$PWD/FFT_C/fft.pri)
include($$PWD/fastmath/fastmath.pri)
include($$PWD/complexNumbers/complex.pri)
include($$PWD/trajectorytracker/trajectorytracker.pri)

SOURCES += \
	$$PWD/imu_worker.cpp


HEADERS += \
	$$PWD/imu_worker.h
    


