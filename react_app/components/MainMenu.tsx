import React from 'react';
import { View, Text, StyleSheet, TouchableOpacity } from 'react-native';
import { StatusBar } from 'expo-status-bar';
import { NavigationProp, useNavigation } from '@react-navigation/native';

// Define the valid routes and their params (if any)
type RootStackParamList = {
  Home: undefined;
  Profile: undefined;
  Settings: undefined;
};

const MainMenu: React.FC = () => {
  // Specify the navigation type so TS knows your valid routes
  const navigation = useNavigation<NavigationProp<RootStackParamList>>();

  // Now the screen parameter is typed as one of the keys in RootStackParamList
  const handleNavigate = (screen: keyof RootStackParamList) => {
    navigation.navigate(screen);
  };

  return (
    <View style={styles.container}>
      <Text style={styles.title}>My Awesome App</Text>
      <TouchableOpacity style={styles.menuButton} onPress={() => handleNavigate('Home')}>
        <Text style={styles.menuButtonText}>Home</Text>
      </TouchableOpacity>
      <TouchableOpacity style={styles.menuButton} onPress={() => handleNavigate('Profile')}>
        <Text style={styles.menuButtonText}>Profile</Text>
      </TouchableOpacity>
      <TouchableOpacity style={styles.menuButton} onPress={() => handleNavigate('Settings')}>
        <Text style={styles.menuButtonText}>Settings</Text>
      </TouchableOpacity>
      <StatusBar style="auto" />
    </View>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#D7f7f7',
    alignItems: 'center',
    justifyContent: 'center',
    padding: 20,
  },
  title: {
    fontSize: 32,
    fontWeight: 'bold',
    marginBottom: 40,
    color: '#333',
  },
  menuButton: {
    backgroundColor: '#4CAF50',
    paddingVertical: 15,
    paddingHorizontal: 40,
    borderRadius: 25,
    marginVertical: 10,
    width: '80%',
    alignItems: 'center',
  },
  menuButtonText: {
    color: '#fff',
    fontSize: 18,
    fontWeight: '600',
  },
});

export default MainMenu;
