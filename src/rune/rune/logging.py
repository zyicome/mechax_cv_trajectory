import logging

logging.basicConfig(
    format='[%(asctime)s][%(levelname)s] %(message)s',
    level=logging.INFO,
    handlers=[
        logging.StreamHandler(),
    ]
)
logger = logging.getLogger('rune')

if __name__ == '__main__':
    logger.info('Hello, world!')
