ls ../output/makespan | ./convert_back # this does both makespan and distance, assuming solutions exist for each of them
#ls ../output/distance | ./convert_back # should be unnecessary (unless we start doing makespan/distance-specific solutions)
nums=$RANDOM
# zip ../zips/upload-$nums.zip ../json_out/*-distance.json ../json_out/*-makespan.json -x ../json_out/\*test\* -j
zip ../zips/upload-$nums-p1.zip ../json_out/[abcdefghij]*-distance.json ../json_out/[jabcdefghi]*-makespan.json -x ../json_out/\*test\* -j
zip ../zips/upload-$nums-p2.zip ../json_out/[klmnop]*-distance.json ../json_out/[klmnop]*-makespan.json -x ../json_out/\*test\* -j
zip ../zips/upload-$nums-p3.zip ../json_out/[qrstuvwxyz]*-distance.json ../json_out/[qrstuvwxyz]*-makespan.json -x ../json_out/\*test\* -j
echo "Output to ../zips/upload-$nums-p[123].zip"
#zip ../zips/upload-distance-$nums.zip ../json_out/*-distance.json -x ../json_out/\*test\* -j
#echo "Output to ../zips/upload-distance-$nums.zip"
#zip ../zips/upload-makespan-$nums.zip ../json_out/*-makespan.json -x ../json_out/\*test\* -j
#echo "Output to ../zips/upload-makespan-$nums.zip"
