// Imports dependencies.
import React, { createContext, useState, useEffect } from 'react';
import { Platform, PermissionsAndroid } from 'react-native';
import { Device } from 'react-native-ble-plx';

// Imports app service.
import BLEService from './BLEService';


//
export const BLEContext = createContext<{
  device: Device | null;
  setDevice: (device: Device | null) => void;
  serviceUUID: string;
  setServiceUUID: (uuid: string) => void;
  characteristicUUID: string;
  setCharacteristicUUID: (uuid: string) => void;
}>({
  device: null,
  setDevice: () => {},
  serviceUUID: '',
  setServiceUUID: () => {},
  characteristicUUID: '',
  setCharacteristicUUID: () => {},
});


//
export const BLEProvider = ({ children }: { children: React.ReactNode }) => {
  const [device, setDevice] = useState<Device | null>(null);
  const [serviceUUID, setServiceUUID] = useState<string>('');                   // Mutable service UUID string.
  const [characteristicUUID, setCharacteristicUUID] = useState<string>('');     // Mutable characteristics UUID string.

    useEffect(() => {

        // Request needed permissions on Android.
        async function requestPermissions() {
            if (Platform.OS === 'android') {
                await PermissionsAndroid.requestMultiple([
                    PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION,
                    PermissionsAndroid.PERMISSIONS.BLUETOOTH_SCAN,
                    PermissionsAndroid.PERMISSIONS.BLUETOOTH_CONNECT,
                ]);
            }
        }
        
        requestPermissions().then(() => {
            BLEService.startScan((foundDevice) => {
                
                // Optionally filter by name or other criteria here.
                if (foundDevice.name && foundDevice.name.includes('pico')) {
                    
                    // Sets found device as webapp BLE device.
                    setDevice(foundDevice);
                    
                    // Stops scan when device has been found.
                    BLEService.stopScan();
                }
            });
        });

        return () => {
            // Optionally you could destroy service on app unmount.
            // BLEService.destroy();            
        };
    }, []);

    return (
      <BLEContext.Provider value={{
        device,
        setDevice,
        serviceUUID,
        setServiceUUID,
        characteristicUUID,
        setCharacteristicUUID,
      }}>
        {children}
      </BLEContext.Provider>
    );
};
