import { Dimensions, StyleSheet } from 'react-native';
import { LineChart } from 'react-native-chart-kit';

import EditScreenInfo from '@/components/EditScreenInfo';
import { Text, View } from '@/components/Themed';

const screenWidth = Dimensions.get("window").width;
const screenHeight = Dimensions.get("window").height;

// Import data from persistent storage solution. (DB)
const data = {
  labels: ["Jan", "Feb", "Mar", "Apr", "May", "Jun"],
  datasets: [
    {
      data: [20, 45, 28, 80, 99, 43],
      strokeWidth: 2, // optional
    },
  ],
};

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

export default function TabTwoScreen() {
  return (
    <View style={styles.container}>
      <Text style={styles.title}>Your Posture Trends</Text>
      <View style={styles.separator} lightColor="#eee" darkColor="rgba(255,255,255,0.1)" />
      
      
      <LineChart 
        data = {data}
        width={screenWidth*0.9}
        height={screenHeight*0.45}
        chartConfig={chartConfig}
        style={styles.chart}
      />
    
    </View>
  );  
}
  // above LineChart
  // <EditScreenInfo path="app/(tabs)/two_trends.tsx" /> 

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
  chart : {
    marginVertical: 8,
    borderRadius: 16,
  },
});
