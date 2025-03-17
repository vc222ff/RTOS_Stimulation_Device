import { Platform } from "react-native";
import type { Device } from "react-native-ble-plx";

export interface IBLEService {
    startScan: (onDeviceFound: (device: any) => void) => void;
    stopScan: () => void;
    destroy: () => void;
}

let BLEService: IBLEService;


if (Platform.OS === 'web') {
    console.warn('BLEService and BLE is not supported on web. Using dummy implementation.')
    BLEService = {
        startScan: (onDeviceFound: (device: any) => void) => {},
        stopScan: () => {},
        destroy: () => {},
    };
} else {
    // Dynamically imports native BLE dependencies.
    const { BleManager } = require('react-native-ble-plx');

    class RealBLEService implements IBLEService {
        private manager: any;

        constructor() {
            this.manager = new BleManager();
        }

        startScan(onDeviceFound: (device: Device) => void) {
            this.manager.startDeviceScan(null, null, (error: any, device: Device) => {
                if (error) {
                    console.error('Error during BLE scan:', error);
                    return;
                }
                if (device) {
                    onDeviceFound(device);
                }
            });
        }

        stopScan() {
            this.manager.stopDeviceScan();
        }
    
        destroy() {
            this.manager.destroy();
        }
    }

    BLEService = new RealBLEService(); 
}


export default BLEService;
