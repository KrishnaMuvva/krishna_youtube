#!/bin/sh

git clone -b 4.27 https://github.com/EpicGames/UnrealEngine

cd UnrealEngine

./Setup.sh

./GenerateProjectFiles.sh

make

echo "Unreal Engine Setup is Done"
