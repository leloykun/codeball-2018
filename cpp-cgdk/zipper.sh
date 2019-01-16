make -C build || exit 1
echo DONE MAKING!

VERSION=$1
zip "versions/MyStrategy_v$VERSION.zip" MyStrategy.h MyStrategy.cpp \
  PointVectors.h RenderUtil.h RenderUtil.cpp Simulation.h SimPredict.cpp \
  Simulation.cpp SimUtils.cpp Entity.h Entity.cpp GeomUtils.h GeomUtils.cpp
echo "done zipping version $VERSION"

cp build/MyStrategy "versions/MyStrategy_v$VERSION"
