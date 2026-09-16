import { HeaderButton } from '@robiolab/native-ui';
import { useKeepAwake } from 'expo-keep-awake';
import { Stack } from 'expo-router';
import * as React from 'react';
import { BackHandler } from 'react-native';
import { ConnectScreen } from '../src/screens/ConnectScreen';
import { DashboardScreen } from '../src/screens/DashboardScreen';
import { disconnect, useEris } from '../src/store/erisStore';

/** Streaming is something you watch: keep the screen awake while it runs. */
function KeepAwake() {
  useKeepAwake();
  return null;
}

export default function Home() {
  const connection = useEris((s) => s.connection);
  const deviceName = useEris((s) => s.deviceName);
  const streaming = useEris((s) => s.streaming);
  const connected = connection === 'configuring' || connection === 'ready';

  // Back from the dashboard disconnects instead of leaving the app.
  React.useEffect(() => {
    if (!connected) return;
    const sub = BackHandler.addEventListener('hardwareBackPress', () => {
      disconnect();
      return true;
    });
    return () => sub.remove();
  }, [connected]);

  return (
    <>
      <Stack.Screen
        options={{
          title: connected ? (deviceName ?? 'Eris BLE') : 'Eris BLE',
          headerRight: connected ? () => <HeaderButton title="Disconnect" onPress={disconnect} /> : undefined,
        }}
      />
      {connected && streaming ? <KeepAwake /> : null}
      {connected ? <DashboardScreen /> : <ConnectScreen />}
    </>
  );
}
