make -C cpp-cgdk/build || exit 1

cd cpp-cgdk
./zipper.sh $1

cd ../
git add .
git commit -m "version $1"
git push
