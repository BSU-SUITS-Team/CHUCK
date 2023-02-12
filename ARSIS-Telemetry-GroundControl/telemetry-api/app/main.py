from fastapi import FastAPI

app = FastAPI()


@app.get("/")
async def root():
    return {"message": "Hello World"}


# TODO: Move this into a locations module
@app.get("/location/")
async def location():
    return {"lat": 10, "lon": 10}


# TODO: Move this into a biometrics module
@app.get("/biometrics/")
async def biometrics():
    return {"hr": 75, "o2": 0.9}
