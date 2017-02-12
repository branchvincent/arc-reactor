PYSIDE_UIC := $(shell locate -b '\pyside-uic')
PYSIDE_RCC := $(shell locate -b '\pyside-rcc')

%_rc.py: %.qrc
	$(PYSIDE_RCC) $< > $@

%.py: %.ui
	$(PYSIDE_UIC) $< > $@

all: front_panel.py resources_rc.py
