import boto3
from botocore.config import Config

def get_s3_client():
    return boto3.client(
        "s3",
        config=Config(s3={"addressing_style": "path"}),
    )
