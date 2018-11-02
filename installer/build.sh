#!/bin/bash

mkdir -p ROOT/tmp/Pulsar2_X2/
cp "../Pulsar2.ui" ROOT/tmp/Pulsar2_X2/
cp "../mountlist Pulsar2.txt" ROOT/tmp/Pulsar2_X2/
cp "../build/Release/libPulsar2.dylib" ROOT/tmp/Pulsar2_X2/

if [ ! -z "$installer_signature" ]; then
# signed package using env variable installer_signature
pkgbuild --root ROOT --identifier org.rti-zone.Pulsar2_X2 --sign "$installer_signature" --scripts Scripts --version 1.0 Pulsar2_X2.pkg
pkgutil --check-signature ./Pulsar2_X2.pkg
else
pkgbuild --root ROOT --identifier org.rti-zone.Pulsar2_X2 --scripts Scritps --version 1.0 Pulsar2_X2.pkg
fi

rm -rf ROOT
