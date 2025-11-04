float LFP(float input, float last_input, float alpha)
{
    return alpha * input + (1 - alpha) * last_input;
}
