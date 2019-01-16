make -C cpp-cgdk/build || exit 1
echo DONE MAKING!

cpp-cgdk/zipper.sh $1
git add .
git commit -m "version $1"
git push
