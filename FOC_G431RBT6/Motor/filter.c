

float LFP(float input, float last_output, float alpha)
{
    return alpha * input + (1 - alpha) * last_output;
}