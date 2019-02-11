/// Copyright (c) CodeBrew GmbH
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.ComponentModel.Composition;
using LabNation.Interfaces;

namespace LabNation.Decoders
{
    [Export(typeof(IProcessor))]
    public class DecoderI2C : IDecoder
    {
        public DecoderDescription Description
        {
            get
            {
                return new DecoderDescription()
                {
                    Name = "CAN HS/FD Decoder",
                    ShortName = "CAN",
                    Author = "Benjamin Bässler CodeBrew GmbH",
                    VersionMajor = 0,
                    VersionMinor = 1,
                    Description = "Decodes CAN HS/FD Frames with arbitration, data and errors.",
                    InputWaveformTypes = new Dictionary<string, Type>() 
                    {
                        { "CAN_H", typeof(float)},
                        { "CAN_L", typeof(float)}
                    },
                    InputWaveformExpectedToggleRates = new Dictionary<string, ToggleRate>() 
                    {
                        { "CAN_H", ToggleRate.High},
                        { "CAN_L", ToggleRate.High}
                    },
                    Parameters = new DecoderParameter[]
                    {
                        new DecoderParameterStrings("Baud", new[] { "125000", "250000", "500000", "1000000"}, "125_000", "Bits per second (baudrate)."),
                    },
                    ContextMenuOrder = new List<string>(new string[] { "In1", "Dum", "In2" })
                };
            }
        }

        public DecoderOutput[] Process(Dictionary<string, Array> inputWaveforms, Dictionary<string, object> parameters, double samplePeriod)
        {
            //name input waveforms for easier usage
            bool[] CAN_H = (bool[])inputWaveforms["CAN_H"];
            bool[] CAN_L = (bool[])inputWaveforms["CAN_L"];

            var selectedBaudrate = int.TryParse((string)parameters["Baud"], out selectedBaudrate);
            double bitLength = 1000.0 / selectedBaudrate;

            //initialize output structure
            List<DecoderOutput> decoderOutputList = new List<DecoderOutput>();

            /// We need to find the threshold between high and low value. Therefore we need to find min and max values
            /// On the edges of a rising or falling signal there can be over/undershoots, this should be ignored as min max
            /// To get rid of this outliers we sort the values and throw away the highest and lowest quartile
            
            // first we need to calculate the value of the differential signal
            var differentialSignal = new List<float>();
            for (int i = 0; i < CAN_H.Length; i++)
            {
                // calculate from differntial signal
                differentialSignal.Add(CAN_H[i] - CAN_L[i]);
            }

            // now lets sort this list
            var sortedDifferentialSignal = new List<float>(differentialSignal);
            sortedDifferentialSignal.Sort();

            // and find the threshold by look in the two quartiles
            float lowerQuartile = sortedDifferentialSignal[Math.Ceiling(sortedDifferentialSignal.Count / 4)];
            float upperQuartile = sortedDifferentialSignal[Math.Ceiling(sortedDifferentialSignal.Count * 3 / 4)];
            float interQuartileRange = upperQuartile - lowerQuartile;
            float threshold = interQuartileRange/2 + lowerQuartile;

            // in case of a to small threshold the meassurement is not very good and we not try to decode it
            // 1 is chosen almost random, but in a perfect world the distance between high and low would be around 2V
            if (interQuartileRange < .6)
            {
                return decoderOutputList.ToArray();
            }

            int lastValueStart = 0;
            // the dominant bit represents the logical 0 (it is inverted)
            bool lastValue = differentialSignal[0] < threshold;

            var bits = new List<Bit>();

            // TODO: write every bit into list even if there is no edge (use bitLength)

            // Now we can decode the highs and lows aka bits
            for (int i = 1; i < differentialSignal.Count; i++)
            {
                bool curValue = differentialSignal[i] < threshold;

                // is this a edge?
                if ( curValue != lastValue)
                {
                    
                    /// is this more than one bit (allow some jitter)
                    /// using here 4% is not supper critical the longest period of consecutive bits of the same kind are 5 maybe in some cases 6
                    int numberOfBits = Math.Ceiling(curBitLength / (bitLength * 0.96));
                    int indexPerBit = (i - lastValueStart) / numberOfBits;

                    for (int j=0; j < numberOfBits; j++ )
                    {
                        if (j + 1 == numberOfBits)
                        {
                            bits.Add(new Bit(lastValueStart, i, lastValue));
                            lastValueStart = i;
                        }
                        else
                        {
                            bits.Add(new Bit(lastValueStart, lastValueStart + indexPerBit, lastValue));
                            lastValueStart = lastValueStart + indexPerBit;
                        }
                    }
                    lastValue = curValue;
                }
            }


            CanField currentField = Unknown;
            int interFrameCounter = 0;
            int stuffBitCounter = 0;
            bool lastBit = false;
            int arbitartionSignificance = 0;

            int stuffingErrorStart = -1;
            int canID;
            byte curDataByte = 0;
            CanMessage curCanMessage;

            posCount = 0;
            

            foreach (Bit bit in bits)
            {

                // count equal following bits to find stuffing bits
                if (lastBit == bit.Value)
                {
                    stuffBitCounter++;

                    if (stuffBitCounter > 5)
                    {
                        curCanMessage.setStuffingErrorDetected(bit.Index);
                        continue;
                    }
                }
                else
                {
                    // is this a stuffing bit?
                    if (stuffBitCounter == 5)
                    {
                        stuffBitCounter = 0;
                        continue;
                    }
                    stuffBitCounter = 0;
                }
                lastBit = bit.Value;

                switch (currentField)
                {
                    case CanField.Unknown:
                        stuffBitCounter = 0;

                        // we wait for 11 consecutive recessive (1) bits to make sure we not start decoding inside of a frame
                        if (bit.Value)
                        {
                            if (++interFrameCounter >= 11)
                            {
                                currentField = CanField.Gap;
                            }
                        }
                        else
                        {
                            // we are not in the void
                            interFrameCounter = 0;
                        }
                        break;

                    case CanField.Gap:
                        stuffBitCounter = 1;
                        stuffingErrorStart = -1;

                        if (!bit.Value)
                        {
                            decoderOutputList.Add(new DecoderOutputEvent(bit.Index, bit.EndIndex, DecoderOutputColor.Orange, "SOF"));
                            currentField = CanField.Arbitration;
                            arbitartionSignificance = 10;
                            curCanMessage.Id = 0;
                            curCanMessage = CanMessage();
                        }
                        break;

                    case CanField.Arbitration:
                        (bit.Value) curCanMessage.Id += 1 << (arbitartionSignificance);
                        arbitartionSignificance--;

                        if (arbitartionSignificance < 0)
                        {
                            currentField = CanField.ReqRemote;
                        }
                        break;

                    case CanField.ReqRemote:
                        curCanMessage.Remote = bit.Value;
                        currentField = CanField.ExtendedFrameBit;
                        break;

                    case CanField.ExtendedFrameBit:
                        curCanMessage.Extended = bit.Value;
                        currentField = CanField.Reserved2;

                        if (curCanMessage.Extended)
                        {
                            currentField = CanField.ExtendedArbitration;
                            arbitartionSignificance = 29;
                        }

                        break;

                    case CanField.ExtendedArbitration:
                        if (bit.Value) curCanMessage.Id += 1 << (arbitartionSignificance);
                        arbitartionSignificance--;

                        if (arbitartionSignificance < 12)
                        {
                            currentField = CanField.RemoteTransmissionBit;
                        }
                        break;

                    case CanField.RemoteTransmissionBit:
                        currentField = CanField.Reserved1;
                        break;

                    case CanField.Reserved1:
                        currentField = CanField.Reserved2;
                        break;

                    case CanField.Reserved2:
                        currentField = CanField.Control;
                        posCount = 0;
                        break;

                    case CanField.Control:
                        if (bit.Value) curCanMessage.Dlc += 1 << (3 - posCount);
                        posCount++;

                        if (posCount == 4)
                        {
                            if (curCanMessage.Dlc > 0) currentField = CanField.Data;
                            else currentField = CanField.CRC;
                            posCount = 0;
                            curDataByte = 0;
                        }
                        break;

                    case CanField.Data:
                        if (bit.Value) curDataByte += 1 << ((7 - posCount) % 8);
                        posCount++;

                        // full byte received
                        if (posCount % 8 == 0 && posCount > 0)
                        {
                            curCanMessage.Data.Add(curDataByte);
                            curDataByte = 0;

                            if (posCount / 8 == curCanMessage.Dlc)
                            {
                                currentField = CanField.CRC;
                                posCount = 0;
                            }
                        }
                        break;

                    case CanField.CRC:
                        if (bit.Value) curCanMessage.Crc += 1 << (15 - posCount);
                        posCount++;

                        if (posCount > 15) currentField = CanField.AckBit;
                        break;

                    case CanField.AckBit:
                        curCanMessage.Ack = bit.Value;
                        currentField = CanField.AckDelimiter;
                        break;

                    case CanField.AckDelimiter:
                        posCount = 0;
                        currentField = CanField.EndOfFrame;
                        break;

                    case CanField.EndOfFrame:
                        posCount++;
                        if (!bit.Value) curCanMessage.setEndOfFrameErrorDetected(bit.Index);

                        if (posCount == 7)
                        {
                            posCount = 0;
                            currentField = CanField.InterFrameSeparation;
                        }
                        break;

                    case CanField.InterFrameSeparation:
                        posCount++;
                        if (!bit.Value) curCanMessage.setIntermissionSilenceErrorDetected(bit.Index);

                        if (posCount == 3)
                        {
                            // this is the end...
                            decoderOutputList.Add(curCanMessage.endOfFrame(bit.EndIndex));
                            currentField = CanField.Gap;
                        }
                        break;
                }

             return decoderOutputList.ToArray();
        }
    }

    private enum CanField
    {
        Unknown,
        Gap,
        Arbitration,
        ReqRemote,
        ExtendedFrameBit,
        ExtendedArbitration,
        RemoteTransmissionBit,
        Reserved1,
        Reserved2,
        Control,
        Data,
        CRC,
        AckBit,
        AckDelimiter,
        EndOfFrame,
        InterFrameSeparation,
    }

    private class CanMessage
    {
            public List Data;
            public int Id;
            public bool Extended { get; private set; }
            public bool Remote { get; private set; }
            public int Dlc { get; private set; }
            public int Crc { get; private set; }
            public bool Ack { get; private set; }
            public bool StuffingError { get; private set; }
            public int StuffingIndexStart { get; private set; }
            public int ArbitrationStart { get; private set; }
            public bool EndOfFrameError { get; private set; }
            public int EndOfFrameErrorIndex { get; private set; }
            public bool IntermissionSilenceError { get; private set; }
            public int IntermissionSilenceErrorIndex { get; private set; }
            public int EndOfFrameIndex { get; private set; }

            private List<DecoderOutput> decoderOutput;

            public CanMessage()
            {
                this.Id = 0;
                this.Extended = false;
                this.Remote = false;
                this.Dlc = 0;
                this.Crc = 0;
                this.Ack = false;
                this.Data = new List<Byte>();
                this.StuffingError = false;
                this.decoderOutput = new List<DecoderOutput>();
            }

            public setStuffingErrorDetected(int index)
            {
                this.StuffingError = true;
                this.StuffingIndexStart = index;
            }

            public setEndOfFrameErrorDetected(int index)
            {
                this.EndOfFrameError = true;
                this.EndOfFrameErrorIndex = index;
            }

            public setIntermissionSilenceErrorDetected(int index)
            {
                this.IntermissionSilenceError = true;
                this.IntermissionSilenceErrorIndex = index;
            }

            public DecoderOutput[] endOfFrame(int endFrameIndex)
            {
                this.EndOfFrameIndex = endFrameIndex;
                // TODO Mark the end
                return decoderOutput.ToArray;
            }
        }

    /// <summary>
    /// The bit. Thanks to robert44 for brining the bit to me
    /// </summary>
    private class Bit
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="Bit"/> class.
        /// </summary>
        /// <param name="index"> The index. </param>
        /// <param name="endIndex"> The length. </param>
        /// <param name="val"> The val. </param>
        public Bit(int index, int endIndex, bool val)
        {
            this.Index = index;
            this.Value = val;
            this.EndIndex = endIndex;
        }

        /// <summary>
        /// Gets the index.
        /// </summary>
        public int Index { get; private set; }

        /// <summary>
        /// Gets the value.
        /// </summary>
        public bool Value { get; private set; }

        /// <summary>
        /// Gets the length.
        /// </summary>
        public int EndIndex { get; private set; }

        /// <summary>
        /// The to string.
        /// </summary>
        /// <returns>
        /// The <see cref="string"/>.
        /// </returns>
        public override string ToString()
        {
            return string.Format("{0},{1},{2}", this.Index, this.EndIndex, this.Value);
        }
    }
}
