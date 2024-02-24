package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;


@I2cDeviceType
@DeviceProperties(name = "TrackBall sensor", description = "a Tracking Ball sensor", xmlTag = "TrackBall")
public class TrackBallDriver extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    short x = 0;
    short y = 0;

    short offsetX = 0;
    short offsetY = 0;


    public short getManufacturerIDRaw() {return 1;}

    //0-255
    public short getX() {
        short dataRawXPositive = readByte(Register.RIGHT);
        short dataRawXNegative = readByte(Register.LEFT);
        x += dataRawXPositive;
        x -= dataRawXNegative;
        return (short) (x - offsetX);
    }
    //0-255
    public short getY() {
        short dataRawYPositive = readByte(Register.UP);
        short dataRawYNegative = readByte(Register.DOWN);
        y += dataRawYPositive;
        y -= dataRawYNegative;
        return (short) (y - offsetY);
    }

    public void reset(){
        offsetX = getX();
        offsetY = getY();
    }


    protected void writeShort(final Register reg, short value) {
        deviceClient.write(reg.ordinal(), TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    //should be signed
    protected byte readByte(Register reg){
        return deviceClient.read8(reg.bVal);
    }

    public short getXByte() {return readShort(Register.RIGHT);}

    public short getYByte() {return readShort(Register.LEFT);}
    public short getUpByte() {return readShort(Register.UP);}
    public short getDownByte() {return readShort(Register.DOWN);}
    public void setGreen() {writeShort(Register.LED_GRN, (short)0xff);}
    public void setRed() {writeShort(Register.LED_RED, (short)0xff);}
    public void setBlue() {writeShort(Register.LED_BLU, (short)0xff);}

    //Config settings

    public enum Register {
        LED_GRN(0x00),
        LED_BLU(0x01),
        LED_WHT(0x02),
        LED_RED(0x03),
        LEFT(0x04),
        RIGHT(0x05),
        UP(0x06),
        DOWN(0x07),
        SWITCH(0x07),
        LAST(SWITCH.bVal);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x0A);

    public TrackBallDriver(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);

        this.setOptimalReadWindow();
        //00001010 or 00001011
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        return false;
    }

    protected void setOptimalReadWindow()
    {

        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.LED_GRN.bVal,
                Register.LAST.bVal - Register.LED_GRN.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName()
    {
        return "TrackBall Sensor";
    }
}