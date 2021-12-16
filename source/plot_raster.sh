#!/bin/sh
if [ "$#" -ne 1 ] || ! [ -f "$1" ]; then
	echo Invalid usage
	exit 1
fi
gimp -i -b "(let* ((image (car (gimp-file-load RUN-NONINTERACTIVE \"$1\" \"$1\"))) (drawable (car (gimp-image-get-active-layer image)))) (gimp-image-convert-grayscale image) (gimp-layer-flatten drawable) (plug-in-edge RUN-NONINTERACTIVE image drawable 10 0 1) (gimp-drawable-levels-stretch drawable) (gimp-drawable-invert drawable TRUE) (gimp-file-save RUN-NONINTERACTIVE image drawable \"$1.pgm\" \"$1.pgm\") (gimp-image-delete image) (gimp-quit 0))"
autotrace --centerline --color-count=2 --output-format=svg --output-file=$1.svg --input-format=pgm $1.pgm
