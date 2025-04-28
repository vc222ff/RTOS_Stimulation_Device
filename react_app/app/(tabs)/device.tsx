// Imports dependencies.
import { FlatList, TouchableOpacity, Button, StyleSheet } from 'react-native';
import React, { useContext, useEffect, useState } from 'react';
import { Device } from 'react-native-ble-plx';
import { Text, View } from '@/components/Themed';

// Imports app services and hooks.
import BLEService from '@/services/BLEService';
import { BLEContext } from '@/services/BLEContext';
import { usePostureData } from '@/hooks/usePostureData';


// Expo tab three function.
export default function DeviceScreen() {
  const { device, setDevice, serviceUUID } = useContext(BLEContext);        // Retrieves the currently connected BLE device.
  const { postureData } = usePostureData(device);                           // Retrieves posture data from device.
  
  const [foundDevices, setFoundDevices] = useState<Device[]>([]);
  const [isScanning, setIsScanning] = useState(false);


  // Starts scanning for devices
  const startScan = () => {
    setFoundDevices([]);
    setIsScanning(true);
    const seen = new Set<string>();
    
    BLEService.startScan((found) => {
      if (!seen.has(found.id)) {
        seen.add(found.id);
        setFoundDevices(prev => [...prev, found]);
      }
    }, { serviceUUID });
  };

  // Stops scan on unmount or after connect
  useEffect(() => {
    return () => BLEService.stopScan();
  }, []);


  // Connects to a BLE-device.
  const connectToDevice = (selected: Device) => {
    BLEService.stopScan();
    setDevice(selected);
    setIsScanning(false);
  };
  

  // Sends baseline calibration BLE-request to BLE-device. 
  const sendCalibrationRequest = async () => {
    if (!device) {
      console.warn('No device is connected.');
      return;
    }
    try {
      // Discovers all UUID services and characteristics on BLE-device.
      await device.discoverAllServicesAndCharacteristics();
      
      // Destination service and characterstic UUIDs.
      const serviceUUID = 'b17a58e4-3f6d-4ae0-a70a-1656a3e59be1';         // PRIMARY_SERVICE, b17a58e4-3f6d-4ae0-a70a-1656a3e59be1 # RX Handle "0x000d".
      const characteristicUUID = 'b17a58e4-3f6d-4ae0-a70a-1656a3e59be1';  // CHARACTERISTIC, b17a58e4-3f6d-4ae0-a70a-1656a3e59be1, WRITE | DYNAMIC, # Defines 128b UUID reference.

      // Formats request string into base64 format.
      const request = 'calibrate';
      const base64String = Buffer.from(request).toString('base64');
      
      // Sends calibration command to BLE-device.
      await device.writeCharacteristicWithResponseForService(serviceUUID, characteristicUUID, base64String);
      console.log('Calibration request string successfully sent to embedded device.');
    } 
    catch (error) {
      console.error('Calibration request failed:', error);
    }
  }


  // Disconnects from BLE-device.
  const disconnect = () => {
    device?.cancelConnection();
    setDevice(null);
  };
  

  return (
    <View style={styles.container}>
      <Text style={styles.title}>Your Device</Text>

      <Text style={{ color: device ? 'green' : 'red' }}>
        {device ? 'Connected to: ' + device.name : 'No BLE device connected'}
      </Text>
      
      {device ? (
        <View>
          <Text>Name: {device.name}</Text>
          <Text>ID: {device.id}</Text>
          <Button title="Calibrate Personal Baseline" onPress={sendCalibrationRequest} />
          <Button title="Disconnect" onPress={disconnect} />
        </View>
      ) : (
        <View style={{ width: '100%' }}>
          <Text style={styles.uuidLabel}>Scanning for Service UUID:</Text>
          <Text style={styles.uuidValue}>{serviceUUID || 'Not set'}</Text>

          <Button title={isScanning ? "Scanning..." : "Scan for Devices"} onPress={startScan} />

          {foundDevices.length > 0 && (
            <FlatList
              data={foundDevices}
              keyExtractor={(item) => item.id}
              renderItem={({ item }) => {
                const isPostureDevice = item.name?.toLowerCase().includes('pico') || false;

                return (
                  <TouchableOpacity
                    style={[
                      styles.deviceItem,
                      isPostureDevice && { backgroundColor: '#d0f0d0' }  // highlight if matched
                    ]}
                    onPress={() => connectToDevice(item)}
                  >
                    <Text style={styles.deviceName}>{item.name || 'Unnamed Device'}</Text>
                    <Text style={styles.deviceId}>{item.id}</Text>
                  </TouchableOpacity>
                );
              }}
            />
          )}
        </View>
      )}

      {postureData && (
        <View style={{ marginTop: 20 }}>
          <Text style={styles.label}>Posture Data</Text>
          <Text>ax1: {postureData.ax1}</Text>
          <Text>ay1: {postureData.ay1}</Text>
          <Text>az1: {postureData.az1}</Text>
        </View>
      )}

    </View>
  );
}

// Stylesheet containing styles for tab three.
const styles = StyleSheet.create({
  container: {
    flex: 1,
    padding: 20,
    alignItems: 'center',
    justifyContent: 'flex-start',
    paddingTop: 40,
  },
  title: {
    fontSize: 22,
    fontWeight: 'bold',
    marginBottom: 24,
    textAlign: 'center',
  },
  separator: {
    marginVertical: 30,
    height: 1,
    width: '80%',
  },
  label: {
    fontSize: 18, 
    marginBottom: 5, 
    fontWeight: '600' 
  },
  deviceItem: {
    padding: 12,
    borderBottomWidth: 1,
    borderColor: '#ccc',
  },
  deviceName: {
    fontSize: 16,
    fontWeight: '500',
  },
  deviceId: {
    fontSize: 12,
    color: '#666',
  },
  uuidLabel: {
    fontSize: 12,
    color: '#666',
    marginTop: 10,
    textAlign: 'center',
  },
  uuidValue: {
    fontSize: 14,
    fontWeight: '600',
    marginBottom: 10,
    textAlign: 'center',
  },
  highlightedDevice: {
    backgroundColor: '#e0f7e9', // light green background
  },
  highlightedText: {
    color: 'green',
    fontWeight: 'bold',
  },
});
