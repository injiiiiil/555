#!/usr/bin/env bash
# if you have modules/esp_idf setup as a submodule, then leave it as a submodule and switch branches

COMMIT="cc3203dc4f087ab41b434afff1ed7520c6d90993"
CHECKOUT="cc3203d"

if [ ! -d modules ]; then
echo "this script needs to be run from the root of your repo, sorry, giving up."
exit 1
fi
echo `ls modules`
cd modules

if [ ! -d esp_idf ]; then
    echo 'did not find modules/esp_idf folder, making it.' ; 
    mkdir -p -v esp_idf
else
    echo 'found modules/esp_idf folder' ; 
fi

echo "looking for submodule or repo..."
if [ `git submodule | grep esp_idf | wc | cut -c1-7` == '1'  ]; then 
    echo "found real submodule, syncing"
    ../Tools/gittools/submodule-sync.sh >/dev/null
else
    echo "esp_idf is NOT a submodule"

    if  [ ! `ls  esp_idf/install.sh 2>/dev/null` ]; then
        echo "found empty IDF, cloning"
        # add esp_idf as almost submodule, depths  uses less space
        git clone -b 'release/v5.3'  https://github.com/espressif/esp-idf.git esp_idf
        git checkout $CHECKOUT
    fi
fi

echo "inspecting possible IDF... "
cd esp_idf
echo `git rev-parse HEAD`
# these are a selection of possible specific commit/s that represent v5.3 branch of the esp_idf 
if [ `git rev-parse HEAD` == '$COMMIT' ]; then 
    echo "IDF version 'release/5.3' found OK, great."; 
else
    echo "looks like an idf, but not v5.3 branch, or wrong commit , trying to switch branch and reflect upstream";
    ../../Tools/gittools/submodule-sync.sh >/dev/null
    git fetch ; git checkout -f release/v5.3 
    git checkout $CHECKOUT

    # retry same as above
    echo `git rev-parse HEAD`
    if [ `git rev-parse HEAD` == '$COMMIT' ]; then 
        echo "IDF version 'release/5.3' found OK, great."; 
        git checkout $CHECKOUT
    fi
fi
cd ../..

cd modules/esp_idf 
git submodule update --init --recursive

echo "apply patch for SOC_RMT_SUPPORT_RX_PINGPONG bug"
sed -i 's/^#define SOC_RMT_SUPPORT_RX_PINGPONG.*/#define SOC_RMT_SUPPORT_RX_PINGPONG 0 \/* esps32s3 does not support PING-PONG feature *\//' ./components/soc/esp32s3/include/soc/soc_caps.h
sed -i 's/^#define SOC_RMT_SUPPORT_RX_PINGPONG.*/#define SOC_RMT_SUPPORT_RX_PINGPONG 0 \/* esps32s2 does not support PING-PONG feature *\//' ./components/soc/esp32s2/include/soc/soc_caps.h

cd ../..
echo "after changing IDF versions [ such as between 4.4 and 5.3 ] you should re-run these in your console:"
echo "./modules/esp_idf/install.sh"
echo "source ./modules/esp_idf/export.sh"
