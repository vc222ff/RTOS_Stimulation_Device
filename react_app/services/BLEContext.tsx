import React, { createContext, useState, useEffect } from 'react';
import { Platform, PermissionsAndroid } from 'react-native';
import { Device } from 'react-native-ble-plx';

import BLEService from './BLEService';

export const BLEContext = createContext<{device: Device | null}>({ device: null});


export const BLEProvider = ({ children }: { children: React.ReactNode }) => {
    const [device, setDevice] = useState<Device | null>(null);

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
        <BLEContext.Provider value = {{ device }}>
            {children}
        </BLEContext.Provider>
    );
};
