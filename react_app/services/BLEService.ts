// Imports dependencies.
import { Buffer } from 'buffer';
import { Platform } from "react-native";
import { BleManager, type Device } from "react-native-ble-plx";

// Imports constants.
import { SERVICE_UUID, CHARACTERISTIC_UUID } from '@/constants/BLEConstants';

// Global instance.
let BLEService: IBLEService;                                        


// Interface definition.
export interface IBLEService {
  startScan: (
    onDeviceFound: (device: Device) => void,
    options?: { serviceUUID?: string }
  ) => void;
  stopScan: () => void;
  sendBLERequest: (device: Device, request: string) => Promise<void>;
  destroy: () => void;
}


// Real implementation for native.
class RealBLEService implements IBLEService {
  private manager: BleManager;

  constructor() {
    this.manager = new BleManager();
  }


  // Starts scanning for nearby BLE-devices.
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

  
  // Stops the BLE scan.
  stopScan() {
    this.manager.stopDeviceScan();
  }


  // BLE-request function for sending commands to BLE-device.
  async sendBLERequest(device: Device, request: string) {
    if (!device) throw new Error('No BLE-device connected');

    // Discovers all UUID services and characteristics on BLE-device.
    await device.discoverAllServicesAndCharacteristics();

    // Formats request string into base64 format.
    const base64String = Buffer.from(request).toString('base64');

    // Sends request command to BLE-device.
    await device.writeCharacteristicWithResponseForService(SERVICE_UUID, CHARACTERISTIC_UUID, base64String);
  }

  
  // Destroys this service after use.
  destroy() {
    this.manager.destroy();
  }
}


// Platform-specific assignment for development in web.
if (Platform.OS === 'web') {
  console.warn('BLE is not supported on web — using dummy BLEService.');
  BLEService = {
    startScan: (onDeviceFound: (device: any) => void, _options?: any) => {},
    stopScan: () => {},
    sendBLERequest: async (device: Device, request: string) => {},
    destroy: () => {},
  };
} else {
  BLEService = new RealBLEService();
}


// Default export service.
export default BLEService;
