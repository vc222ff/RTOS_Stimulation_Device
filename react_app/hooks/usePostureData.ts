// Imports dependencies.
import { useContext, useEffect, useState } from 'react';
import { Buffer } from 'buffer';
import { Device } from 'react-native-ble-plx';

// Imports app service.
import { BLEContext } from '@/services/BLEContext';

// Global variables-
const postureKeys: (keyof PostureData)[] = [
    'ax1', 'ay1', 'az1',
    'gx1', 'gy1', 'gz1',
    't1',
    'ax2', 'ay2', 'az2',
    'gx2', 'gy2', 'gz2',
    't2'
];


//
export interface PostureData {
    ax1?: number; ay1?: number; az1?: number; 
    gx1?: number; gy1?: number; gz1?: number;
    t1?: number; 
    ax2?: number; ay2?: number; az2?: number; 
    gx2?: number; gy2?: number; gz2?: number; 
    t2?: number;
}

//
export interface TimedPostureData extends PostureData {
    timestamp: string;
}


//
export function usePostureData(device: Device | null) {
  const { serviceUUID, characteristicUUID } = useContext(BLEContext);

  const [postureData, setPostureData] = useState<PostureData | null>(null);
  const [history, setHistory] = useState<TimedPostureData[]>([]);


  useEffect(() => {
    if (!device || !serviceUUID || !characteristicUUID) return;

    const setupNotification = async () => {
      try {
        await device.discoverAllServicesAndCharacteristics();

        device.monitorCharacteristicForService(
          serviceUUID,
          characteristicUUID,
          (error, characteristic) => {
            if (error) {
              console.error('BLE notification error:', error);
              return;
            }

            if (characteristic?.value) {
              const decoded = Buffer.from(characteristic.value, 'base64').toString('utf-8');
              const parsed = parsePostureData(decoded);

              const entry = {
                ...parsed,
                timestamp: new Date().toISOString(),
              };

              setHistory(prev => {
                const next = [...prev, entry];
                return next.length > 50 ? next.slice(-50) : next;
              });

              setPostureData(entry);
            }
          }
        );
      } catch (err) {
        console.error('BLE setup error:', err);
      }
    };

    setupNotification();
  }, [device, serviceUUID, characteristicUUID]);

  return { postureData, history };
}


// Parses strings like: ax1: 123 | ay1: 456 | ...
// 
function parsePostureData(data: string): PostureData {
    const result: PostureData = {};
    const lines = data.split('\n');
    for (const line of lines) {
      const pairs = line.split('|');
      for (const pair of pairs) {
        const [key, val] = pair.split(':').map(s => s.trim());
        if (key && val && postureKeys.includes(key as keyof PostureData)) {
          result[key as keyof PostureData] = parseFloat(val);
        }
      }
    }
    return result;
  }

