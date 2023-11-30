#
# This file is generated by ModusToolbox during the 'make getlibs' operation
# Any edits to this file will be lost the next time the library manager is run or
# the next time 'make getlibs' is run.
#
# List of local libraries



# The search paths for the included middleware
SEARCH_btsdk-ble=../mtb_shared/wiced_btsdk/dev-kit/libraries/btsdk-ble/release-v4.4.0
SEARCH_btsdk-host-peer-apps-mesh=../mtb_shared/wiced_btsdk/tools/btsdk-host-peer-apps-mesh/release-v4.2.1
SEARCH_btsdk-mesh=../mtb_shared/wiced_btsdk/dev-kit/libraries/btsdk-mesh/release-v4.2.0
SEARCH_btsdk-ota=../mtb_shared/wiced_btsdk/dev-kit/libraries/btsdk-ota/release-v4.2.0
SEARCH_btsdk-peer-apps-ble=../mtb_shared/wiced_btsdk/tools/btsdk-peer-apps-ble/release-v3.3.0
SEARCH_btsdk-peer-apps-ota=../mtb_shared/wiced_btsdk/tools/btsdk-peer-apps-ota/release-v3.3.0
SEARCH_TARGET_CYBT-413055-EVAL=../mtb_shared/wiced_btsdk/dev-kit/bsp/TARGET_CYBT-413055-EVAL/release-v3.3.0
SEARCH_20719B2=../mtb_shared/wiced_btsdk/dev-kit/baselib/20719B2/release-v4.4.2
SEARCH_btsdk-common=../mtb_shared/wiced_btsdk/dev-kit/libraries/btsdk-common/release-v4.2.1
SEARCH_btsdk-include=../mtb_shared/wiced_btsdk/dev-kit/btsdk-include/release-v4.4.2
SEARCH_btsdk-tools=../mtb_shared/wiced_btsdk/dev-kit/btsdk-tools/release-v3.3.0
SEARCH_btsdk-utils=../mtb_shared/wiced_btsdk/tools/btsdk-utils/release-v4.4.1
SEARCH_core-make=../mtb_shared/core-make/release-v1.9.1

# Search libraries added to build
SEARCH+=$(SEARCH_btsdk-ble)
SEARCH+=$(SEARCH_btsdk-host-peer-apps-mesh)
SEARCH+=$(SEARCH_btsdk-mesh)
SEARCH+=$(SEARCH_btsdk-ota)
SEARCH+=$(SEARCH_btsdk-peer-apps-ble)
SEARCH+=$(SEARCH_btsdk-peer-apps-ota)
SEARCH+=$(SEARCH_TARGET_CYBT-413055-EVAL)
SEARCH+=$(SEARCH_20719B2)
SEARCH+=$(SEARCH_btsdk-common)
SEARCH+=$(SEARCH_btsdk-include)
SEARCH+=$(SEARCH_btsdk-tools)
SEARCH+=$(SEARCH_btsdk-utils)
SEARCH+=$(SEARCH_core-make)

-include $(CY_INTERNAL_APP_PATH)/importedbsp.mk
COMPONENTS += MW_BTSDK_BLE
COMPONENTS += MW_BTSDK_HOST_PEER_APPS_MESH
COMPONENTS += MW_BTSDK_MESH
COMPONENTS += MW_BTSDK_OTA
COMPONENTS += MW_BTSDK_PEER_APPS_BLE
COMPONENTS += MW_BTSDK_PEER_APPS_OTA
COMPONENTS += MW_TARGET_CYBT_413055_EVAL
COMPONENTS += MW_20719B2
COMPONENTS += MW_BTSDK_COMMON
COMPONENTS += MW_BTSDK_INCLUDE
COMPONENTS += MW_BTSDK_TOOLS
COMPONENTS += MW_BTSDK_UTILS
COMPONENTS += MW_CORE_MAKE
