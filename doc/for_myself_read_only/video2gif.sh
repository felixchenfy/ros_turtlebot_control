#!/usr/bin/bash

ffmpeg \
    -i demo.mp4 \
    -r 15 \
    -vf scale=512:-1 \
    -ss 00:00:00 -to 00:00:31 \
    output_compressed.gif

# The output gif is compressed.
# The only defect is that the output framerate cannot be adjusted.

# You may try this tool to compress the gif again.
# sudo apt install gifsicle
gifsicle -i output_compressed.gif -O5 --colors 256 -o demo.gif
