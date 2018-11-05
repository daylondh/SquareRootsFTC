package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import java.util.ArrayList;
import java.util.List;


public class PixyManager {
    I2cDeviceSynch pixy;
    List<Byte> desiredSignatures = new ArrayList<Byte>();

    public PixyManager(I2cDeviceSynch cam)
    {
        pixy = cam;
        desiredSignatures.add((byte) 1);
    }

    public List<PixySignature> Run()
    {
        pixy.engage();
        return CheckAllSigs(desiredSignatures);
    }

    private List<PixySignature> CheckAllSigs(List<Byte> desiredSignatures) {
        List<PixySignature> toReturn = new ArrayList<>();
        for (Byte b : desiredSignatures) {
            byte[] signature = pixy.read(0x50 + b, 5);
            int x = 0xff & signature[1];
            int y = 0xff & signature[2];
            int width = 0xff & signature[3];
            int height = 0xff & signature[4];
            PixySignature sig = new PixySignature();
            sig.sigNum = b;
            sig.x = x;
            sig.y = y;
            sig.width = width;
            sig.height = height;
            toReturn.add(sig);
        }
        return toReturn;
    }
}