// Imports dependencies.
import { useContext } from 'react';
import { Dimensions, StyleSheet } from 'react-native';
import { LineChart } from 'react-native-chart-kit';
import { Text, View } from '@/components/Themed';

// Imports app service and hook.
import { BLEContext } from '@/services/BLEContext';
import { usePostureData } from '@/hooks/usePostureData'; 


// Global variables.
const screenWidth = Dimensions.get("window").width;
const screenHeight = Dimensions.get("window").height;

// Time format for data timestamps.
const formatTime = (iso: string) => {
  const date = new Date(iso);
  return `${date.getMinutes()}:${String(date.getSeconds()).padStart(2, '0')}`;
};

// Configuration settings for graph.
const chartConfig = {
  backgroundColor: "#e26a00",
  backgroundGradientFrom: "#fb8c00",
  backgroundGradientTo: "#ffa726",
  decimalPlaces: 2, // optional, defaults to 2dp
  color: (opacity = 1) => `rgba(255, 255, 255, ${opacity})`,
  labelColor: (opacity = 1) => `rgba(255, 255, 255, ${opacity})`,
  style: {
    borderRadius: 16,
  },
  propsForDots: {
    r: "6",
    strokeWidth: "2",
    stroke: "#ffa726",
  },
};


// TODO: Import data from persistent storage solution. (DB)

// Expo tab two function.
export default function TabTwoScreen() {
  
  // Function-scoped variables.
  const { device } = useContext(BLEContext);
  const { history } = usePostureData(device);
  
  // Maps history onto data points for graph representation.
  const data = {
    labels: history.map((p, i) =>
      i % 10 === 0 ? formatTime(p.timestamp) : '' // space labels out
    ),
    datasets: [
      {
        data: history.map(p => p.az1 ?? 0),
        strokeWidth: 2,
      },
    ],
  };

  return (
    <View style={styles.container}>
      <Text style={styles.title}>Your Posture Trends</Text>

      {device ? (
        <LineChart 
          data={data}
          width={screenWidth * 0.9}
          height={screenHeight * 0.45}
          chartConfig={chartConfig}
          style={styles.chart}
        />
      ) : (
        <Text>No BLE-device currently connected to your mobile device</Text>
      )}
    </View>
  );  
}


// Stylesheet containing styles for tab two.
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
  chart : {
    marginVertical: 8,
    borderRadius: 16,
  },
});
