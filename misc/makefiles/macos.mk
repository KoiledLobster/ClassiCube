SOURCE_DIRS := src src/macos third_party/bearssl
BUILD_DIR	:= build/macos
TARGET 		:= ClassiCube
DIST_NAME	:= ClassiCube

CFLAGS	:= -fvisibility=hidden -fno-ident -Wno-error=deprecated-declarations
LDFLAGS	:= -rdynamic
LIBS 	:= -framework Security -framework Cocoa -framework OpenGL -framework IOKit -lobjc
# Metal graphics backend (make darwin BUILD_METAL=1)
ifdef BUILD_METAL
	BUILD_DIR    := build/macos-metal
	SOURCE_DIRS  += src/darwin
	CFLAGS += -DCC_GFX_BACKEND=CC_GFX_BACKEND_METAL
	LIBS   += -framework Metal -framework QuartzCore
endif

include misc/makefiles/common_config.mk


default: $(TARGET).app


#---------------------------------------------------------------------------------
# executable generation
#---------------------------------------------------------------------------------
# macOS app bundle
$(TARGET).app : $(TARGET)
	mkdir -p $@/Contents/MacOS
	mkdir -p $@/Contents/Resources
	cp $< $@/Contents/MacOS/$<
	cp misc/macOS/Info.plist   $@/Contents/Info.plist
	cp misc/macOS/appicon.icns $@/Contents/Resources/appicon.icns

include misc/makefiles/common_build.mk


#---------------------------------------------------------------------------------
# common targets
#---------------------------------------------------------------------------------
include misc/makefiles/common_targets.mk

dist: $(TARGET)
	$(call DIST_PKG_INIT_DEFAULT,$(TARGET))
	$(call DIST_PKG_BUILD_TAR,$(TARGET))
