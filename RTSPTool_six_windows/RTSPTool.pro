#-------------------------------------------------
#
# Project created by QtCreator 2014-04-30T10:42:34
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RTSPTool
TEMPLATE = app


SOURCES += main.cpp\
        frmmain.cpp \
    qffmpeg.cpp \
    rtspthread.cpp

HEADERS  += frmmain.h \
    qffmpeg.h \
    rtspthread.h

FORMS    += frmmain.ui

#INCLUDEPATH +=  /usr/local/ffmpeg/include
#FFMPEG_INCLUDE  = /usr/local/ffmpeg/include
#FFMPEG_LIB      = /usr/local/ffmpeg/lib

#LIBS += $$FFMPEG_LIB/libavcodec.so      \
#        $$FFMPEG_LIB/libavdevice.so     \
#        $$FFMPEG_LIB/libavfilter.so     \
#        $$FFMPEG_LIB/libavformat.so     \
#        $$FFMPEG_LIB/libavutil.so       \
#        $$FFMPEG_LIB/libswresample.so   \
#        $$FFMPEG_LIB/libswscale.so

INCLUDEPATH += /usr/local/ffmpeg-build/include
LIBS += /usr/local/ffmpeg-build/lib/libavformat.so \
        /usr/local/ffmpeg-build/lib/libavdevice.so \
        /usr/local/ffmpeg-build/lib/libavcodec.so \
        /usr/local/ffmpeg-build/lib/libavfilter.so \
        /usr/local/ffmpeg-build/lib/libavutil.so    \
        /usr/local/ffmpeg-build/lib/libswscale.so \
        /usr/local/ffmpeg-build/lib/libswresample.so

MOC_DIR=temp/moc
RCC_DIR=temp/rcc
UI_DIR=temp/ui
OBJECTS_DIR=temp/obj
DESTDIR=bin

win32:RC_FILE=main.rc

RESOURCES += \
    res.qrc
