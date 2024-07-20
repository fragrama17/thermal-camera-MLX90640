using System.Device.I2c;

namespace ThermalCamera;

public class ThermalCamera
{
    private static readonly I2cDevice Device = I2cBus.Create(1).CreateDevice(0x33);
    private readonly ParamsMlx _mlx;
    public const int TotPixels = 768;
    public const int TotColumns = 32;
    public const int TotRows = 24;
    private const int TotAuxData = 64;
    private const int FrameDataError = -8;

    private const ushort StatusRegister = 0x8000,
        ControlRegister = 0x800D,
        ConfigurationRegister = 0x800F,
        RamStartRegister = 0x0400,
        RamEndRegister = 0x06FF,
        AuxDataStartAddress = 0x0700,
        EePromStartAddress = 0x2400;

    public ThermalCamera()
    {
        _mlx = new ParamsMlx();

        var eepromData = new ushort[832];
        ReadWordsFromRegister(EePromStartAddress, eepromData);

        ExtractVddParameters(eepromData);
        ExtractPtatParameters(eepromData);
        ExtractGainParameters(eepromData);
        ExtractTgcParameters(eepromData);
        ExtractResolutionParameters(eepromData);
        ExtractKsTaParameters(eepromData);
        ExtractKsToParameters(eepromData);
        ExtractAlphaParameters(eepromData);
        ExtractOffsetParameters(eepromData);
        ExtractKtaPixelParameters(eepromData);
        ExtractKvPixelParameters(eepromData);
        ExtractCpParameters(eepromData);
        ExtractCilcParameters(eepromData);
        ExtractDeviatingPixels(eepromData);

        Console.WriteLine("successfully extracted params from eeprom");
    }

    private void ExtractVddParameters(ushort[] eepromData)
    {
        byte kVdd = (byte)((eepromData[51] & 0xFF00) >> 8);

        short vdd25 = (short)(eepromData[51] & 0x00FF);
        vdd25 = (short)(((vdd25 - 256) << 5) - 8192);

        _mlx.KVdd = (short)(32 * kVdd);
        _mlx.Vdd25 = vdd25;
    }

    private void ExtractPtatParameters(ushort[] eeData)
    {
        float kvPtat = (eeData[50] & 0xFC00) >> 10;
        if (kvPtat > 31)
        {
            kvPtat -= 64;
        }

        kvPtat /= 4096;

        float ktPtat = eeData[50] & 0x03FF;
        if (ktPtat > 511)
        {
            ktPtat -= 1024;
        }

        ktPtat /= 8;

        var vPtat25 = eeData[49];

        var alphaPtat = (float)((eeData[16] & 0xF000) / Math.Pow(14, 2) + 8.0f);

        _mlx.KvPtat = kvPtat;
        _mlx.KtPtat = ktPtat;
        _mlx.VPtat25 = vPtat25;
        _mlx.AlphaPtat = alphaPtat;
    }

    private void ExtractGainParameters(ushort[] eeData)
    {
        _mlx.GainEe = (short)eeData[48];
    }

    private void ExtractTgcParameters(ushort[] eeData)
    {
        _mlx.Tgc = (eeData[60] & 0x00FF) / 32.0f;
    }

    private void ExtractResolutionParameters(ushort[] eeData)
    {
        _mlx.ResolutionEe = (byte)((eeData[56] & 0x3000) >> 12);
    }

    private void ExtractKsTaParameters(ushort[] eeData)
    {
        _mlx.KsTa = (eeData[60] & 0x00FF) / 8192.0f;
    }

    private void ExtractKsToParameters(ushort[] eeData)
    {
        var step = (byte)(((eeData[63] & 0x3000) >> 12) * 10);

        _mlx.Ct[0] = -40;
        _mlx.Ct[1] = 0;
        // WTF ?
        // _mlx.Ct[2] = (short)((eeData[63] & 0x00F0) >> 4);
        // _mlx.Ct[3] = (short)((eeData[63] & 0x00F0) >> 4);
        _mlx.Ct[2] = (short)(_mlx.Ct[2] * step);
        _mlx.Ct[3] = (short)(_mlx.Ct[2] + _mlx.Ct[3] * step);
        _mlx.Ct[4] = 400;

        var ksToScale = (eeData[63] & 0x000F) + 8;
        ksToScale = (int)(1UL << ksToScale);

        _mlx.KsTo[0] = (eeData[61] & 0x00FF) / (float)ksToScale;
        _mlx.KsTo[1] = ((eeData[61] & 0xFF00) >> 8) / (float)ksToScale;
        _mlx.KsTo[2] = (eeData[62] & 0x00FF) / (float)ksToScale;
        _mlx.KsTo[3] = ((eeData[62] & 0xFF00) >> 8) / (float)ksToScale;
        _mlx.KsTo[4] = (float)-0.0002;
    }

    private void ExtractAlphaParameters(ushort[] eeData)
    {
        int[] accRow = new int[24];
        int[] accColumn = new int[32];
        int p;
        float[] alphaTemp = new float[TotPixels];


        var accRemScale = (byte)(eeData[32] & 0x000F);
        var accColumnScale = (byte)((eeData[32] & 0x00F0) >> 4);
        var accRowScale = (byte)((eeData[32] & 0x0F00) >> 8);
        var alphaScale = (byte)(((eeData[32] & 0xF000) >> 12) + 30);
        int alphaRef = eeData[33];

        for (int i = 0; i < 6; i++)
        {
            p = i * 4;
            accRow[p + 0] = eeData[34 + i] & 0x000F;
            accRow[p + 1] = (eeData[34 + i] & 0x00F0) >> 4;
            accRow[p + 2] = (eeData[34 + i] & 0x0F00) >> 8;
            accRow[p + 3] = (eeData[34 + i] & 0xF000) >> 12;
        }

        for (int i = 0; i < TotRows; i++)
        {
            if (accRow[i] > 7)
            {
                accRow[i] -= 16;
            }
        }

        for (int i = 0; i < 8; i++)
        {
            p = i * 4;
            accColumn[p + 0] = eeData[40 + i] & 0x000F;
            accColumn[p + 1] = (eeData[40 + i] & 0x00F0) >> 4;
            accColumn[p + 2] = (eeData[40 + i] & 0x0F00) >> 8;
            accColumn[p + 3] = (eeData[40 + i] & 0xF000) >> 12;
        }

        for (int i = 0; i < TotColumns; i++)
        {
            if (accColumn[i] > 7)
            {
                accColumn[i] -= 16;
            }
        }

        for (int i = 0; i < TotRows; i++)
        {
            for (int j = 0; j < TotColumns; j++)
            {
                p = 32 * i + j;
                alphaTemp[p] = (eeData[64 + p] & 0x03F0) >> 4;
                if (alphaTemp[p] > 31)
                {
                    alphaTemp[p] -= 64;
                }

                alphaTemp[p] *= 1 << accRemScale;
                alphaTemp[p] = alphaRef + (accRow[i] << accRowScale) + (accColumn[j] << accColumnScale) +
                               alphaTemp[p];
                alphaTemp[p] /= (float)Math.Pow(alphaScale, 2);
                alphaTemp[p] -= _mlx.Tgc * (_mlx.CpAlpha[0] + _mlx.CpAlpha[1]) / 2;
                alphaTemp[p] = (float)(0.000001 / alphaTemp[p]);
            }
        }

        var temp = alphaTemp[0];
        for (int i = 1; i < TotPixels; i++)
        {
            if (alphaTemp[i] > temp)
            {
                temp = alphaTemp[i];
            }
        }

        alphaScale = 0;
        while (temp < 32767.4)
        {
            temp *= 2;
            alphaScale = (byte)(alphaScale + 1);
        }

        for (int i = 0; i < TotPixels; i++)
        {
            temp = (float)(alphaTemp[i] * Math.Pow(alphaScale, 2));
            _mlx.Alpha[i] = (ushort)(temp + 0.5);
        }

        _mlx.AlphaScale = alphaScale;
    }

    private void ExtractOffsetParameters(ushort[] eeData)
    {
        int[] occRow = new int[24];
        int[] occColumn = new int[32];
        int p;

        byte occRemScale = (byte)(eeData[16] & 0x000F);
        byte occColumnScale = (byte)((eeData[16] & 0x00F0) >> 4);
        byte occRowScale = (byte)((eeData[16] & 0x0F00) >> 8);
        short offsetRef = (short)eeData[17];

        for (int i = 0; i < 6; i++)
        {
            p = i * 4;
            occRow[p + 0] = eeData[18] & 0x000F;
            occRow[p + 1] = (eeData[18] & 0x00F0) >> 4;
            occRow[p + 2] = (eeData[18 + i] & 0x0F00) >> 8;
            occRow[p + 3] = (eeData[18 + i] & 0xF000) >> 12;
        }

        for (int i = 0; i < TotRows; i++)
        {
            if (occRow[i] > 7)
            {
                occRow[i] -= 16;
            }
        }

        for (int i = 0; i < 8; i++)
        {
            p = i * 4;
            occColumn[p + 0] = eeData[24 + i] & 0x000F;
            occColumn[p + 1] = (eeData[24 + i] & 0x00F0) >> 4;
            occColumn[p + 2] = (eeData[24 + i] & 0x0F00) >> 8;
            occColumn[p + 3] = (eeData[24 + i] & 0xF000) >> 12;
        }

        for (int i = 0; i < TotColumns; i++)
        {
            if (occColumn[i] > 7)
            {
                occColumn[i] -= 16;
            }
        }

        for (int i = 0; i < TotRows; i++)
        {
            for (int j = 0; j < TotColumns; j++)
            {
                p = 32 * i + j;
                _mlx.Offset[p] = (short)((eeData[64 + p] & 0xFC00) >> 10);
                if (_mlx.Offset[p] > 31)
                {
                    _mlx.Offset[p] -= 64;
                }

                _mlx.Offset[p] *= (short)(1 << occRemScale);
                _mlx.Offset[p] = (short)(offsetRef + (occRow[i] << occRowScale) + (occColumn[j] << occColumnScale) +
                                         _mlx.Offset[p]);
            }
        }
    }

    private void ExtractKtaPixelParameters(ushort[] eeData)
    {
        sbyte[] ktaRc = new sbyte[4];
        float[] ktaTemp = new float[768];

        ktaRc[0] = (sbyte)((eeData[54] & 0xFF00) >> 8);
        ktaRc[2] = (sbyte)((eeData[54] & 0x00FF) >> 8);
        ktaRc[1] = (sbyte)((eeData[54] & 0xFF00) >> 8);
        ktaRc[3] = (sbyte)((eeData[54] & 0x00FF) >> 8);

        byte ktaScale1 = (byte)((eeData[56] & 0x00F0) >> 4);
        byte ktaScale2 = (byte)(eeData[56] & 0x000F);

        for (int i = 0; i < TotRows; i++)
        {
            for (int j = 0; j < TotColumns; j++)
            {
                var p = 32 * i + j;
                byte split = (byte)(2 * (p / 32 - p / 64 * 2) + p % 2);
                ktaTemp[p] = (eeData[64 + p] & 0x000E) >> 1;
                if (ktaTemp[p] > 3)
                {
                    ktaTemp[p] -= 8;
                }

                ktaTemp[p] *= 1 << ktaScale2;
                ktaTemp[p] = ktaRc[split] + ktaTemp[p];
                ktaTemp[p] /= (float)Math.Pow(ktaScale1, 2);
            }
        }

        float temp = Math.Abs(ktaTemp[0]);
        for (int i = 1; i < TotPixels; i++)
        {
            if (Math.Abs(ktaTemp[i]) > temp)
            {
                temp = Math.Abs(ktaTemp[i]);
            }
        }

        ktaScale1 = 0;
        while (temp < 63.4)
        {
            temp *= 2;
            ktaScale1 = (byte)(ktaScale1 + 1);
        }

        for (int i = 0; i < TotPixels; i++)
        {
            temp = (float)(ktaTemp[i] * Math.Pow(ktaScale1, 2));
            if (temp < 0)
            {
                _mlx.Kta[i] = (sbyte)(temp - 0.5);
            }
            else
            {
                _mlx.Kta[i] = (sbyte)(temp + 0.5);
            }
        }

        _mlx.KtaScale = ktaScale1;
    }

    private void ExtractKvPixelParameters(ushort[] eeData)
    {
        sbyte[] kvT = new sbyte[4];
        float[] kvTemp = new float[TotPixels];

        var kvRoCo = (sbyte)((eeData[52] & 0xF000) >> 12);
        if (kvRoCo > 7)
        {
            kvRoCo = (sbyte)(kvRoCo - 16);
        }

        kvT[0] = kvRoCo;

        var kvReCo = (sbyte)((eeData[52] & 0x0F00) >> 8);
        if (kvReCo > 7)
        {
            kvReCo = (sbyte)(kvReCo - 16);
        }

        kvT[2] = kvReCo;

        var kvRoCe = (sbyte)((eeData[52] & 0x00F0) >> 4);
        if (kvRoCe > 7)
        {
            kvRoCe = (sbyte)(kvRoCe - 16);
        }

        kvT[1] = kvRoCe;

        var kvReCe = (sbyte)(eeData[52] & 0x000F);
        if (kvReCe > 7)
        {
            kvReCe = (sbyte)(kvReCe - 16);
        }

        kvT[3] = kvReCe;

        var kvScale = (byte)((eeData[56] & 0x0F00) >> 8);


        for (int i = 0; i < TotRows; i++)
        {
            for (int j = 0; j < TotColumns; j++)
            {
                var p = 32 * i + j;
                var split = (byte)(2 * (p / 32 - p / 64 * 2) + p % 2);
                kvTemp[p] = kvT[split];
                kvTemp[p] /= (float)Math.Pow(kvScale, 2);
            }
        }

        var temp = Math.Abs(kvTemp[0]);
        for (int i = 1; i < TotPixels; i++)
        {
            if (Math.Abs(kvTemp[i]) > temp)
            {
                temp = Math.Abs(kvTemp[i]);
            }
        }

        kvScale = 0;
        while (temp < 63.4)
        {
            temp *= 2;
            kvScale = (byte)(kvScale + 1);
        }

        for (int i = 0; i < TotPixels; i++)
        {
            temp = (float)(kvTemp[i] * Math.Pow(kvScale, 2));
            if (temp < 0)
            {
                _mlx.Kv[i] = (sbyte)(temp - 0.5);
            }
            else
            {
                _mlx.Kv[i] = (sbyte)(temp + 0.5);
            }
        }

        _mlx.KvScale = kvScale;
    }

    private void ExtractCpParameters(ushort[] eeData)
    {
        float[] alphaSp = new float[2];
        short[] offsetSp = new short[2];

        byte alphaScale = (byte)(((eeData[32] & 0xF000) >> 12) + 27);

        offsetSp[0] = (short)(eeData[58] & 0x03FF);
        if (offsetSp[0] > 511)
        {
            offsetSp[0] = (short)(offsetSp[0] - 1024);
        }

        // offsetSp[1] = (short)((eeData[58] & 0xFC00) >> 10);
        if (offsetSp[1] > 31)
        {
            offsetSp[1] = (short)(offsetSp[1] - 64);
        }

        offsetSp[1] = (short)(offsetSp[1] + offsetSp[0]);

        alphaSp[0] = eeData[57] & 0x03FF;
        if (alphaSp[0] > 511)
        {
            alphaSp[0] -= 1024;
        }

        alphaSp[0] /= (float)Math.Pow(alphaScale, 2);

        // alphaSp[1] = (eeData[57] & 0xFC00) >> 10;
        if (alphaSp[1] > 31)
        {
            alphaSp[1] -= 64;
        }

        alphaSp[1] = (1 + alphaSp[1] / 128) * alphaSp[0];

        float cpKta = eeData[59] & 0x00FF;

        byte ktaScale1 = (byte)(((eeData[56] & 0x00F0) >> 4) + 8);
        _mlx.CpKta = (float)(cpKta / Math.Pow(ktaScale1, 2));

        float cpKv = (eeData[59] & 0xFF00) >> 8;

        int kvScale = (eeData[56] & 0x0F00) >> 8;
        _mlx.CpKv = (float)(cpKv / Math.Pow(kvScale, 2));

        _mlx.CpAlpha[0] = alphaSp[0];
        _mlx.CpAlpha[1] = alphaSp[1];
        _mlx.CpOffset[0] = offsetSp[0];
        _mlx.CpOffset[1] = offsetSp[1];
    }

    private void ExtractCilcParameters(ushort[] eeData)
    {
        float[] ilChessC = new float[3];

        byte calibrationModeEe = (byte)((eeData[10] & 0x0800) >> 4);
        calibrationModeEe = (byte)(calibrationModeEe ^ 0x80);

        ilChessC[0] = (eeData[53] & 0x003F);
        if (ilChessC[0] > 31)
        {
            ilChessC[0] -= 64;
        }

        ilChessC[0] /= 16.0f;

        ilChessC[1] = (eeData[53] & 0x07C0) >> 6;
        if (ilChessC[1] > 15)
        {
            ilChessC[1] -= 32;
        }

        ilChessC[1] /= 2.0f;

        ilChessC[2] = (eeData[53] & 0xF800) >> 11;
        if (ilChessC[2] > 15)
        {
            ilChessC[2] -= 32;
        }

        ilChessC[2] /= 8.0f;

        _mlx.CalibrationModeEe = calibrationModeEe;
        _mlx.IlChessC[0] = ilChessC[0];
        _mlx.IlChessC[1] = ilChessC[1];
        _mlx.IlChessC[2] = ilChessC[2];
    }

    private int ExtractDeviatingPixels(ushort[] eeData)
    {
        ushort pixCnt;
        ushort brokenPixCnt = 0;
        ushort outlierPixCnt = 0;
        int warn = 0;

        for (pixCnt = 0; pixCnt < 5; pixCnt++)
        {
            _mlx.BrokenPixels[pixCnt] = 0xFFFF;
            _mlx.OutlierPixels[pixCnt] = 0xFFFF;
        }

        pixCnt = 0;
        while (pixCnt < TotPixels && brokenPixCnt < 5 && outlierPixCnt < 5)
        {
            if (eeData[pixCnt + 64] == 0)
            {
                _mlx.BrokenPixels[brokenPixCnt] = pixCnt;
                brokenPixCnt = (ushort)(brokenPixCnt + 1);
            }
            else if ((eeData[pixCnt + 64] & 0x0001) != 0)
            {
                _mlx.OutlierPixels[outlierPixCnt] = pixCnt;
                outlierPixCnt = (ushort)(outlierPixCnt + 1);
            }

            pixCnt = (ushort)(pixCnt + 1);
        }

        if (brokenPixCnt > 4)
        {
            warn = -3;
        }
        else if (outlierPixCnt > 4)
        {
            warn = -4;
        }
        else if ((brokenPixCnt + outlierPixCnt) > 4)
        {
            warn = -5;
        }
        else
        {
            int i;
            for (pixCnt = 0; pixCnt < brokenPixCnt; pixCnt++)
            {
                for (i = pixCnt + 1; i < brokenPixCnt; i++)
                {
                    warn = CheckAdjacentPixels(_mlx.BrokenPixels[pixCnt], _mlx.BrokenPixels[i]);
                    if (warn != 0)
                    {
                        return warn;
                    }
                }
            }

            for (pixCnt = 0; pixCnt < outlierPixCnt; pixCnt++)
            {
                for (i = pixCnt + 1; i < outlierPixCnt; i++)
                {
                    warn = CheckAdjacentPixels(_mlx.OutlierPixels[pixCnt], _mlx.OutlierPixels[i]);
                    if (warn != 0)
                    {
                        return warn;
                    }
                }
            }

            for (pixCnt = 0; pixCnt < brokenPixCnt; pixCnt++)
            {
                for (i = 0; i < outlierPixCnt; i++)
                {
                    warn = CheckAdjacentPixels(_mlx.BrokenPixels[pixCnt], _mlx.BrokenPixels[i]);
                    if (warn != 0)
                    {
                        return warn;
                    }
                }
            }
        }


        return warn;
    }

    private static int CheckAdjacentPixels(ushort pix1, ushort pix2)
    {
        ushort lp1 = (ushort)(pix1 >> 5);
        ushort lp2 = (ushort)(pix2 >> 5);
        ushort cp1 = (ushort)(pix1 - (lp1 << 5));
        ushort cp2 = (ushort)(pix2 - (lp2 << 5));

        var pixPosDif = lp1 - lp2;
        if (pixPosDif is <= -2 or >= 2) return 0;

        pixPosDif = cp1 - cp2;
        if (pixPosDif is > -2 and < 2)
        {
            return -6;
        }

        return 0;
    }

    public static async Task<int> GetFrameData(ushort[] frameData)
    {
        ushort dataReady = 0;
        ushort statusWord = 0;
        ushort[] auxData = new ushort[64];
        byte cnt;

        while (dataReady == 0)
        {
            statusWord = ReadWordFromRegister(StatusRegister);
            dataReady = (ushort)((statusWord >> 3) & 0b1);
        }

        Console.WriteLine("data ready ! :D");

        int error = await WriteWordToRegister(StatusRegister, 0x0030);
        if (error != 0)
        {
            return error;
        }

        ReadWordsFromRegister(RamStartRegister, frameData);

        Console.WriteLine("successfully read frame data from ram !");

        ReadWordsFromRegister(AuxDataStartAddress, auxData);

        Console.WriteLine("successfully read aux data from aux register !");

        var controlWord = ReadWordFromRegister(ControlRegister);

        frameData[832] = controlWord;
        frameData[833] = statusWord;

        error = ValidateFrameData(frameData);
        if (error != 0)
        {
            Console.WriteLine("frame validation failed");
            return error;
        }

        Console.WriteLine("successfully validated frame data");

        error = ValidateAuxData(auxData);
        if (error != 0)
        {
            Console.WriteLine("aux validation failed");
            return frameData[833];
        }

        for (cnt = 0; cnt < TotAuxData; cnt++)
        {
            frameData[cnt + TotPixels] = auxData[cnt];
        }

        Console.WriteLine("successfully validated aux data");

        return frameData[833];
    }

    public async Task GetImage(float[] frame)
    {
        float emissivity = 0.95F;
        ushort[] frameData = new ushort[834];

        for (int i = 0; i < 2; i++) // first page 1, then page 2
        {
            int status = await GetFrameData(frameData);
            if (status < 0)
            {
                throw new IOException("error while getting data frame");
            }

            var tr = GetTa(frameData) - 8;
            CalculateTo(frameData, emissivity, tr, frame);
        }
    }

    // FIXME mlx C version return only zeros
    // public async Task GetImage(float[] result)
    // {
    //     var frameData = new ushort[834]; // array containing all the data we need
    //     var error = await GetFrameData(frameData);
    //     if (error < 0)
    //     {
    //         Console.WriteLine($"failed to get data frame, error code: {error}");
    //         return;
    //     }
    //
    //     float vdd = GetVdd(frameData);
    //     float ta = GetTa(frameData);
    //     float gain = (float)_mlx.GainEe / (short)frameData[778];
    //     float[] irDataCp = new float[2];
    //     float ktaScale = (float)Math.Pow(2, _mlx.KtaScale);
    //     float kvScale = (float)Math.Pow(2, _mlx.KvScale);
    //     byte mode = (byte)((frameData[832] & (1UL << 12)) >> 5);
    //     ushort subPage = frameData[833];
    //
    //     irDataCp[0] = (short)frameData[776] * gain;
    //     irDataCp[1] = (short)frameData[808] * gain;
    //
    //     irDataCp[0] -= _mlx.CpOffset[0] * (1 + _mlx.CpKta * (ta - 25)) * (1 + _mlx.CpKv * (vdd - 3.3f));
    //     if (mode == _mlx.CalibrationModeEe)
    //     {
    //         irDataCp[1] -= _mlx.CpOffset[1] * (1 + _mlx.CpKta * (ta - 25)) *
    //                        (1 + _mlx.CpKv * (vdd - 3.3f));
    //     }
    //     else
    //     {
    //         irDataCp[1] -= (_mlx.CpOffset[1] + _mlx.IlChessC[0]) * (1 + _mlx.CpKta * (ta - 25)) *
    //                        (1 + _mlx.CpKv * (vdd - 3.3f));
    //     }
    //
    //     for (int pixelNumber = 0; pixelNumber < TotPixels; pixelNumber++)
    //     {
    //         int ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2;
    //         int chessPattern = ilPattern ^ (pixelNumber - (pixelNumber / 2) * 2);
    //         int conversionPattern =
    //             ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) *
    //             (1 - 2 * ilPattern);
    //
    //         int pattern = mode == 0 ? ilPattern : chessPattern;
    //
    //         if (pattern != frameData[833]) continue;
    //
    //         float irData = (short)frameData[pixelNumber] * gain;
    //         float kta = _mlx.Kta[pixelNumber] / ktaScale;
    //         float kv = _mlx.Kv[pixelNumber] / kvScale;
    //
    //         irData -= _mlx.Offset[pixelNumber] * (1 + kta * (ta - 25)) * (1 + kv * (vdd - 3.3f));
    //         if (mode != _mlx.CalibrationModeEe)
    //         {
    //             irData += _mlx.IlChessC[2] * (2 * ilPattern - 1) - _mlx.IlChessC[1] * conversionPattern;
    //         }
    //
    //         irData -= _mlx.Tgc * irDataCp[subPage];
    //         float alphaCompensated = _mlx.Alpha[pixelNumber];
    //         float image = irData * alphaCompensated;
    //         result[pixelNumber] = image;
    //     }
    // }

    void CalculateTo(ushort[] frameData, float emissivity, float tr, float[] result)
    {
        float[] irDataCp = new float[2];
        float[] alphaCorrR = new float[4];

        var subPage = frameData[833];
        float vdd = GetVdd(frameData);
        float ta = GetTa(frameData);

        float ta4 = (float)(ta + 273.15);
        ta4 *= ta4;
        ta4 *= ta4;
        float tr4 = (float)(tr + 273.15);
        tr4 *= tr4;
        tr4 *= tr4;
        var taTr = tr4 - (tr4 - ta4) / emissivity;

        float ktaScale = (float)Math.Pow(_mlx.KtaScale, 2);
        float kvScale = (float)Math.Pow(_mlx.KvScale, 2);
        float alphaScale = (float)Math.Pow(_mlx.AlphaScale, 2);

        alphaCorrR[0] = 1 / (1 + _mlx.KsTo[0] * 40);
        alphaCorrR[1] = 1;
        alphaCorrR[2] = 1 + _mlx.KsTo[1] * _mlx.Ct[2];
        alphaCorrR[3] = alphaCorrR[2] * (1 + _mlx.KsTo[2] * (_mlx.Ct[3] - _mlx.Ct[2]));

        //------------------------- Gain calculation -----------------------------------    
        var gain = (float)_mlx.GainEe / (short)frameData[778];

        //------------------------- To calculation -------------------------------------    
        byte mode = (byte)((frameData[832] & (1UL << 12)) >> 5);

        irDataCp[0] = (short)frameData[776] * gain;
        irDataCp[1] = (short)frameData[808] * gain;

        irDataCp[0] -= (float)(_mlx.CpOffset[0] * (1 + _mlx.CpKta * (ta - 25)) * (1 + _mlx.CpKv * (vdd - 3.3)));
        if (mode == _mlx.CalibrationModeEe)
        {
            irDataCp[1] = (float)(irDataCp[1] -
                                  _mlx.CpOffset[1] * (1 + _mlx.CpKta * (ta - 25)) * (1 + _mlx.CpKv * (vdd - 3.3)));
        }
        else
        {
            irDataCp[1] = (float)(irDataCp[1] - (_mlx.CpOffset[1] + _mlx.IlChessC[0]) * (1 + _mlx.CpKta * (ta - 25)) *
                (1 + _mlx.CpKv * (vdd - 3.3)));
        }

        for (int pixelNumber = 0; pixelNumber < TotPixels; pixelNumber++)
        {
            sbyte ilPattern = (sbyte)(pixelNumber / 32 - (pixelNumber / 64) * 2);
            sbyte chessPattern = (sbyte)(ilPattern ^ (pixelNumber - (pixelNumber / 2) * 2));
            sbyte conversionPattern =
                (sbyte)(((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) *
                        (1 - 2 * ilPattern));

            var pattern = mode == 0 ? ilPattern : chessPattern;

            if (pattern != frameData[833]) continue;

            float irData = (short)frameData[pixelNumber] * gain;

            var kta = _mlx.Kta[pixelNumber] / ktaScale;
            var kv = _mlx.Kv[pixelNumber] / kvScale;
            irData -= (float)(_mlx.Offset[pixelNumber] * (1 + kta * (ta - 25)) * (1 + kv * (vdd - 3.3)));

            if (mode != _mlx.CalibrationModeEe)
            {
                irData = irData + _mlx.IlChessC[2] * (2 * ilPattern - 1) - _mlx.IlChessC[1] * conversionPattern;
            }

            irData -= _mlx.Tgc * irDataCp[subPage];
            irData /= emissivity;

            float alphaCompensated = (float)(0.000001 * alphaScale / _mlx.Alpha[pixelNumber]);
            alphaCompensated *= 1 + _mlx.KsTa * (ta - 25);

            var sx = alphaCompensated * alphaCompensated * alphaCompensated * (irData + alphaCompensated * taTr);
            sx = (float)(Math.Sqrt(Math.Sqrt(sx)) * _mlx.KsTo[1]);

            var to =
                (float)(Math.Sqrt(Math.Sqrt(irData / (alphaCompensated * (1 - _mlx.KsTo[1] * 273.15) + sx) + taTr)) -
                        273.15);

            sbyte range;
            if (to < _mlx.Ct[1])
            {
                range = 0;
            }
            else if (to < _mlx.Ct[2])
            {
                range = 1;
            }
            else if (to < _mlx.Ct[3])
            {
                range = 2;
            }
            else
            {
                range = 3;
            }

            to = (float)(Math.Sqrt(Math.Sqrt(irData /
                             (alphaCompensated * alphaCorrR[range] *
                              (1 + _mlx.KsTo[range] * (to - _mlx.Ct[range]))) + taTr))
                         - 273.15);

            result[pixelNumber] = to;
        }
    }

    private float GetTa(ushort[] frameData)
    {
        var vdd = GetVdd(frameData);

        var ptat = (short)frameData[800];

        var ptatArt = (float)(ptat / (ptat * _mlx.AlphaPtat + (short)frameData[768]) * Math.Pow(18, 2));

        var ta = (float)(ptatArt / (1 + _mlx.KvPtat * (vdd - 3.3)) - _mlx.VPtat25);
        ta = ta / _mlx.KtPtat + 25;

        return ta;
    }

    private float GetVdd(ushort[] frameData)
    {
        var resolutionRam = (ushort)((frameData[832] & (~(~0 << 2) << 10)) >> 10);
        var resolutionCorrection = (float)(Math.Pow(_mlx.ResolutionEe, 2) / Math.Pow(resolutionRam, 2));
        var vdd = (float)((resolutionCorrection * frameData[810] - _mlx.Vdd25) / _mlx.KVdd + 3.3);

        return vdd;
    }

    private static int ValidateFrameData(ushort[] frameData)
    {
        byte line = 0;

        for (int i = 0; i < TotPixels; i += TotColumns)
        {
            if (frameData[i] == 0x7FFF && line % 2 == frameData[833]) return FrameDataError;
            line++;
        }

        return 0;
    }

    private static int ValidateAuxData(ushort[] auxData)
    {
        if (auxData[0] == 0x7FFF) return FrameDataError;

        for (int i = 8; i < TotAuxData; i++)
        {
            if (auxData[i] == 0x7FFF) return FrameDataError;
        }

        return 0;
    }

    private static void ReadWordsFromRegister(ushort register, ushort[] words)
    {
        var frameBuffer = new byte[words.Length * 2];

        ReadFromRegister(register, frameBuffer);

        // Convert bytes to ushort
        for (int i = 0; i < words.Length; i++)
        {
            words[i] = BitConverter.ToUInt16(frameBuffer, i * 2);
        }
    }

    private static async Task<byte> WriteWordToRegister(ushort register, ushort word)
    {
        var cmd = new byte[4];
        cmd[0] = (byte)(register >> 8);
        cmd[1] = (byte)(register & 0xFF);
        cmd[2] = (byte)(word >> 8);
        cmd[3] = (byte)(word & 0xFF);
        byte[] dataCheck = { 0 };

        Device.Write(cmd);
        await Task.Delay(1);

        ReadFromRegister(register, dataCheck);

        return dataCheck[0];
    }

    private static ushort ReadWordFromRegister(ushort register)
    {
        var wordBuffer = new byte[2];

        ReadFromRegister(register, wordBuffer);

        return (ushort)((wordBuffer[0] << 8) | wordBuffer[1]);
    }

    private static void ReadFromRegister(ushort register, byte[] readBuffer)
    {
        var registerBuffer = new byte[2];
        registerBuffer[0] = (byte)(register >> 8);
        registerBuffer[1] = (byte)(register & 0xFF);

        Device.WriteRead(registerBuffer, readBuffer);
    }

    private sealed class ParamsMlx
    {
        public short KVdd;
        public short Vdd25;
        public float KvPtat;
        public float KtPtat;
        public ushort VPtat25;
        public float AlphaPtat;
        public short GainEe;
        public float Tgc;
        public float CpKv;
        public float CpKta;
        public byte ResolutionEe;
        public byte CalibrationModeEe;
        public float KsTa;
        public readonly float[] KsTo = new float[5];
        public readonly short[] Ct = new short[5];
        public readonly ushort[] Alpha = new ushort[768];
        public byte AlphaScale;
        public readonly short[] Offset = new short[768];
        public readonly sbyte[] Kta = new sbyte[768];
        public byte KtaScale;
        public readonly sbyte[] Kv = new sbyte[768];
        public byte KvScale;
        public readonly float[] CpAlpha = new float[2];
        public readonly short[] CpOffset = new short[2];
        public readonly float[] IlChessC = new float[3];
        public readonly ushort[] BrokenPixels = new ushort[5];
        public readonly ushort[] OutlierPixels = new ushort[5];
    }
}