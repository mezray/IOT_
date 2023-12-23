function Decoder(bytes, port) {
    var decoded = {};
    
    decoded.bytes = bytes
    decoded.empty = bytes.length == 1
  
    
    var temperatureRaw = (bytes[0] << 8) | bytes[1];
    decoded.temperature = temperatureRaw / 100.0;
  
    var hallSensor = (bytes[2] << 8) | bytes[3];

    decoded.isDoorOpen = hallSensor > 50 ? true : false;

  
    // Traitement des valeurs signées
    decoded.a_x = bytes[4] > 127 ? (bytes[4] - 256) : bytes[4];
    decoded.a_y = bytes[5] > 127 ? (bytes[5] - 256) : bytes[5];
    decoded.a_z = bytes[6] > 127 ? (bytes[6] - 256) : bytes[6];
    
    decoded.a_x /= 100;
    decoded.a_y /= 100;
    decoded.a_z /= 100;
    
    if(decoded.a_x == null){
      decoded.a_x = Math.random();
    }
    
    if(decoded.a_y == null){
      decoded.a_y = Math.random();
    }
    
    if(decoded.a_z == null){
      decoded.a_z = Math.random();
    }
  
    // fake data for testing random values between 0 and 1
    decoded.g_x = Math.random();
    decoded.g_y = Math.random();
    decoded.g_z = Math.random();

    // fake data for testing random values between 48 and 50
    decoded.latitude = random(48, 50);
    
    // fake data for testing random values between 2 and 4
    decoded.longitude = random(2, 4);

    return decoded;
}

function random(min, max) {
    return Math.random() * (max - min) + min;
}
