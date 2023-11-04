const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const extend = require('zigbee-herdsman-converters/lib/extend');
const e = exposes.presets;
const ea = exposes.access;


const fzLocal = {
    temp_meas: {
        cluster: 'msTemperatureMeasurement',
        type: ['write'],
        convert: (model, msg, publish, options, meta) => {
            const temperature = parseFloat(msg.data['measuredValue']) / 100.0;
            // const property = postfixWithEndpointName('temperature', msg, model, meta);
            return {
                "temperature": temperature
            };
        },
    },

    pm: {
        cluster: '65522',
        type: ['attributeReport'],
        convert: (model, msg, publish, options, meta) => {
            const pm2 = parseInt(msg.data['0']);


            result = {}

            if('0' in msg.data) {
                result['pm2'] = parseInt(msg.data['0']);
            }

            if('1' in msg.data) {
                result['pm1'] = parseInt(msg.data['0']);
            }

            if('2' in msg.data) {
                result['pm10'] = parseInt(msg.data['0']);
            }

            return result
        },
    }
};

const definition = {
    zigbeeModel: ['MODEL_ZIGBEE_V'], // The model ID from: Device with modelID 'lumi.sens' is not supported.
    model: 'MODEL_ZIGBEE_V', // Vendor model number, look on the device for a model number
    vendor: 'DURKA', // Vendor of the device (only used for documentation and startup logging)
    description: 'Durka is now going to zigbee devices', // Description of the device, copy from vendor site. (only used for documentation and startup logging)
    fromZigbee: [fz.on_off, fzLocal.temp_meas, fz.temperature, fz.humidity, fz.pressure], // We will add this later
    toZigbee: [tz.on_off], // Should be empty, unless device can be controlled (e.g. lights, switches).
    exposes: [e.switch(), e.temperature(), e.humidity(), e.pressure()], // Defines what this device exposes, used for e.g. Home Assistant discovery and in the frontend
    
    configure: async (device, coordinatorEndpoint, logger) => {
        const endpoint = device.getEndpoint(1);
        await reporting.bind(endpoint, coordinatorEndpoint, ['genOnOff']);
        await reporting.onOff(endpoint);
    },
};

module.exports = definition;