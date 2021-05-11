echo "### Running Merge Driver for file $4 ###"
cd build
#./build.sh
echo ../$3 | ./try_save
echo ../$2 | ./try_save
echo ../$1 | ./try_save
cd ${OLDPWD}
cp $4 $2 # overwrite the temp file as well (I think this is what git wants?)
echo "### End Merge Driver ###"

exit 0
