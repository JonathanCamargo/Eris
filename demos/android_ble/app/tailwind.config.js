const robiolab = require('@robiolab/native-ui/tailwind');

module.exports = {
  content: ['./app/**/*.{js,jsx,ts,tsx}', './src/**/*.{js,jsx,ts,tsx}', ...robiolab.content],
  // Brand colors, radius and the :root color variables all come from @robiolab/native-ui.
  presets: [require('nativewind/preset'), robiolab],
};
