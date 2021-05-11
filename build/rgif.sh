# note: 1280x720 must be the same as what's inside src/vis/gif.h
#./build.sh && ./gif_vis | ffmpeg -y -f rawvideo -pixel_format gbrp -video_size 1280x720 -i - -c:v h264 -pix_fmt yuv420p video.mp4
./build.sh && ./gif_vis | ffmpeg -y -f rawvideo -pixel_format gbrp -video_size 1280x720 -i - -filter:v scale=640x360 -c:v h264 -pix_fmt yuv420p video.mp4
