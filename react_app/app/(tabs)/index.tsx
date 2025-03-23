// Imports dependencies.
import { StyleSheet, Image } from 'react-native';

// Imports app component.
import { Text, View } from '@/components/Themed';


// Expo tab one (index) function.
export default function TabOneScreen() {
  return (
    <View style={styles.container}>
    <Text style={styles.title}>Welcome to the Companion App!</Text>
    <Text style={styles.text}>/nThis app is developed for the Posture Correction Wearable Device</Text>

    <Image source={require('@/assets/images/device.png')} style={styles.image}/>
    <View style={styles.separator} lightColor="#eee" darkColor="rgba(255,255,255,0.1)" />
    
    </View>
)};


// Stylesheet containing styles for tab one (index).
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
  text: {
    fontSize: 16,
    fontWeight: 'normal',
  },
  separator: {
    marginVertical: 30,
    height: 1,
    width: '80%',
  },
  image: {
    height: '70%',
    width: '45%',

  }
});
  