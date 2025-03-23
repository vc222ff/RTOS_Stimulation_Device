// Imports dependencies.
import { useContext, useState } from 'react';
import { Button, StyleSheet, TextInput } from 'react-native';

// Imports app component and service.
import { Text, View } from '@/components/Themed';
import { BLEContext } from '@/services/BLEContext';


// Expo tab four function.
export default function SettingsScreen() {
  const { 
    serviceUUID, 
    setServiceUUID, 
    characteristicUUID, 
    setCharacteristicUUID 
  } = useContext(BLEContext);

  const [localService, setLocalService] = useState(serviceUUID);
  const [localChar, setLocalChar] = useState(characteristicUUID);


  return (
    <View style={styles.container}>
      <Text style={styles.title}>BLE Configuration</Text>

      <Text style={styles.label}>Service UUID</Text>
      <TextInput
        style={styles.input}
        placeholder="Enter UUID here"
        value={localService}
        onChangeText={setLocalService}
        autoCapitalize="none"
        autoCorrect={false}
      />
      <Text style={styles.currentValue}>Current: {serviceUUID || 'Not set'}</Text>

      <Text style={styles.label}>Characteristic UUID</Text>
      <TextInput
        style={styles.input}
        placeholder="Enter UUID here"
        value={localChar}
        onChangeText={setLocalChar}
        autoCapitalize="none"
        autoCorrect={false}
      />
      <Text style={styles.currentValue}>Current: {characteristicUUID || 'Not set'}</Text>


      <View style={styles.buttonContainer}>
        <Button
          title="Save UUIDs"
          onPress={() => {
            setServiceUUID(localService);
            setCharacteristicUUID(localChar);
          }}
        />
      </View>
    </View>
  );
}


// Stylesheet containing styles for tab four.
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
    fontSize: 16,
    marginTop: 12,
    marginBottom: 4,
    color: '#333',
  },
  input: {
    borderWidth: 1,
    borderColor: '#ccc',
    borderRadius: 8,
    paddingHorizontal: 12,
    paddingVertical: 8,
    fontSize: 16,
  },
  currentValue: {
    fontSize: 12,
    color: '#777',
    marginBottom: 12,
    paddingTop: 8,
  },
  buttonContainer: {
    marginTop: 24,
    borderRadius: 8,
    overflow: 'hidden',
  },
});
