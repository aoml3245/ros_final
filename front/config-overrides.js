const webpack = require('webpack');

module.exports = function override(config, env) {
  config.resolve.fallback = {
    fs: false,
    path: false,
    os: false
  };
  
  return config;
};