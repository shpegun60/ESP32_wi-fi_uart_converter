INCLUDEPATH += $$PWD
DEPENDPATH += $$PWD	

SOURCES += \
	$$PWD/ahrs_top.cpp \
	$$PWD/ahrs.c \
        $$PWD/coordinate_ahrs.c\
        $$PWD/updater_ahrs.c


HEADERS += \
	$$PWD/ahrs_top.h \
        $$PWD/ahrs.h \
	$$PWD/ahrs_data.h\
	$$PWD/coordinate_ahrs.h\
        $$PWD/updater_ahrs.h
    


