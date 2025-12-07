// Unit tests for core services
// Testing the API service base functionality

import ApiService from '../src/services/api';

// Mock fetch for testing
global.fetch = jest.fn();

describe('ApiService', () => {
  beforeEach(() => {
    (global.fetch as jest.Mock).mockClear();
  });

  it('should perform a GET request', async () => {
    const mockResponse = { data: 'test' };
    (global.fetch as jest.Mock).mockResolvedValue({
      ok: true,
      json: () => Promise.resolve(mockResponse),
    });

    const result = await ApiService.get('/test');
    
    expect(global.fetch).toHaveBeenCalledWith(
      'https://api.physicalai-textbook.com/v1/test',
      {
        method: 'GET',
        headers: expect.any(Headers),
      }
    );
    expect(result).toEqual(mockResponse);
  });

  it('should perform a POST request', async () => {
    const postData = { name: 'test' };
    const mockResponse = { id: '1', name: 'test' };
    (global.fetch as jest.Mock).mockResolvedValue({
      ok: true,
      json: () => Promise.resolve(mockResponse),
    });

    const result = await ApiService.post('/test', postData);
    
    expect(global.fetch).toHaveBeenCalledWith(
      'https://api.physicalai-textbook.com/v1/test',
      {
        method: 'POST',
        headers: expect.any(Headers),
        body: JSON.stringify(postData),
      }
    );
    expect(result).toEqual(mockResponse);
  });

  it('should handle errors correctly', async () => {
    (global.fetch as jest.Mock).mockResolvedValue({
      ok: false,
      status: 404,
      json: () => Promise.resolve({ error: { message: 'Not found' } }),
    });

    await expect(ApiService.get('/test')).rejects.toThrow('Not found');
  });
});