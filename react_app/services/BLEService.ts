// Imports dependencies.
import { Platform } from "react-native";
import { BleManager, type Device } from "react-native-ble-plx";


// Global instance
let BLEService: IBLEService;


// Interface definition
export interface IBLEService {
  startScan: (
    onDeviceFound: (device: Device) => void,
    options?: { serviceUUID?: string }
  ) => void;
  stopScan: () => void;
  destroy: () => void;
}


// Real implementation for native
class RealBLEService implements IBLEService {
  private manager: BleManager;

  constructor() {
    this.manager = new BleManager();
  }

  startScan(
    onDeviceFound: (device: Device) => void,
    options?: { serviceUUID?: string }
  ) {
    const scanFilter = options?.serviceUUID ? [options.serviceUUID] : null;

    this.manager.startDeviceScan(scanFilter, null, (error, device) => {
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

// Platform-specific assignment
if (Platform.OS === 'web') {
  console.warn('BLE is not supported on web — using dummy BLEService.');
  BLEService = {
    startScan: (onDeviceFound: (device: any) => void, _options?: any) => {},
    stopScan: () => {},
    destroy: () => {},
  };
} else {
  BLEService = new RealBLEService();
}

// Default export service.
export default BLEService;
