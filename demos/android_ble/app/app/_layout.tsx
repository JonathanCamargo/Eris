import '../global.css';

import { headerTheme } from '@robiolab/native-ui';
import { Stack } from 'expo-router';
import { StatusBar } from 'expo-status-bar';
import * as React from 'react';
import { GestureHandlerRootView } from 'react-native-gesture-handler';
import { SafeAreaProvider } from 'react-native-safe-area-context';

export default function RootLayout() {
  return (
    <GestureHandlerRootView style={{ flex: 1 }}>
      <SafeAreaProvider>
        {/* Navy header chrome, so light status-bar icons. */}
        <StatusBar style="light" />
        <Stack screenOptions={headerTheme} />
      </SafeAreaProvider>
    </GestureHandlerRootView>
  );
}
