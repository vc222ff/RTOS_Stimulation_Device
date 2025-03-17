import { StyleSheet } from 'react-native';
import { useContext } from 'react';

import EditScreenInfo from '@/components/EditScreenInfo';
import { Text, View } from '@/components/Themed';
import { BLEContext } from '@/services/BLEContext';


export default function TabThreeScreen() {
  // Retrieves the currently connected BLE device.
  const { device } = useContext(BLEContext);

  return (
    <View style={styles.container}>
      <Text style={styles.title}>Your Device</Text>
      <View style={styles.separator} lightColor="#eee" darkColor="rgba(255,255,255,0.1)" />
      <View style={{ flex:1, alignItems: 'center', justifyContent: 'center'}}>
        <Text style={{ fontSize: 18 }}>
           {device ? `Connected to ${device.name}` : 'Scanning for BLE devices...'}
           </Text>
      </View>
      
      <EditScreenInfo path="app/(tabs)/three_device.tsx" />
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    alignItems: 'center',
    justifyContent: 'center',
  },
  title: {
    fontSize: 25,
    fontWeight: 'bold',
  },
  separator: {
    marginVertical: 30,
    height: 1,
    width: '80%',
  },
});
