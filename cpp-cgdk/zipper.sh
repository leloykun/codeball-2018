VERSION=$1
zip "versions/MyStrategy_v$VERSION.zip" MyStrategy.h MyStrategy.cpp PointVectors.h \
  RenderUtil.cpp RenderUtil.h SimPredict.cpp Simulation.cpp Simulation.h \
  SimUtils.cpp
echo "done zipping version $VERSION"

cp build/MyStrategy "versions/MyStrategy_v$VERSION"
